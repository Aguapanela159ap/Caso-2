import os
import csv
from math import radians, sin, cos, sqrt, atan2
from itertools import product
from pyomo.environ import *



#directorio base
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

#Leer csv
coords = {}      
dem_dict = {}    
cap_i_dict = {}  

with open(os.path.join(BASE_DIR, 'clients.csv'), newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        cid = row['ClientID']
        dem_dict[cid] = int(row['Demand'])
        coords[cid]   = (float(row['Longitude']), float(row['Latitude']))

with open(os.path.join(BASE_DIR, 'depots.csv'), newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        did = row['DepotID']
        cap_i_dict[did] = 1000
        coords[did]     = (float(row['Longitude']), float(row['Latitude']))

# csv vehiculo
cap_k_dict = {}
rng_k_dict = {}
with open(os.path.join(BASE_DIR, 'vehicles.csv'), newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        vid = row['VehicleID']
        cap_k_dict[vid] = int(row['Capacity'])
        rng_k_dict[vid] = float(row['Range'])

# Haversine
def haversine(lon1, lat1, lon2, lat2):
    R = 6371.0
    lon1, lat1, lon2, lat2 = map(radians, (lon1, lat1, lon2, lat2))
    dlon, dlat = lon2 - lon1, lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))

#Crear conjuntos 
I = list(cap_i_dict.keys())
J = list(dem_dict.keys())
K = list(cap_k_dict.keys())
N = I + J

# calcular distancias
d_data = {}
for i, j in product(N, N):
    if i != j:
        lon1, lat1 = coords[i]
        lon2, lat2 = coords[j]
        d_data[(i, j)] = haversine(lon1, lat1, lon2, lat2)

# rango max de vehiculo
max_rng = max(rng_k_dict.values())

# arcos factibles
A = [(i, j) for (i, j), dist in d_data.items() if dist <= max_rng]

#Costos globales
Ft, Cm, Pf, alpha = 5000, 700, 15000, 1

# modelo

model = ConcreteModel()
model.I = Set(initialize=I)
model.J = Set(initialize=J)
model.K = Set(initialize=K)
model.N = Set(initialize=N)
model.A = Set(initialize=A, within=model.N * model.N)

model.d     = Param(model.A, initialize=d_data, within=PositiveReals)
model.dem   = Param(model.J, initialize=dem_dict)
model.cap_k = Param(model.K, initialize=cap_k_dict)
model.rng_k = Param(model.K, initialize=rng_k_dict)
model.cap_i = Param(model.I, initialize=cap_i_dict)

model.Ft    = Param(initialize=Ft)
model.Cm    = Param(initialize=Cm)
model.Pf    = Param(initialize=Pf)
model.alpha = Param(initialize=alpha)

# var
model.x = Var(model.A, model.K, domain=Binary)
model.q = Var(model.J, model.K, domain=NonNegativeIntegers)
model.z = Var(model.I, model.K, domain=Binary)
model.u = Var(model.J, domain=NonNegativeReals)
model.y = Var(model.J, model.K, domain=Binary)

# Objetivo
def obj_rule(m):
    return sum(m.x[i, j, k] * m.d[i, j] * (m.Ft + m.Cm + m.Pf)
               for (i, j) in m.A for k in m.K)
model.OBJ = Objective(rule=obj_rule, sense=minimize)

# Restricciones
def r1_demanda(m, j): return sum(m.q[j, k] for k in m.K) == m.dem[j]
model.demanda = Constraint(model.J, rule=r1_demanda)

def r2_capacity_on_y(m, k):
    return sum(m.dem[j] * m.y[j,k] for j in m.J) <= m.cap_k[k]
model.capacity_on_y = Constraint(model.K, rule=r2_capacity_on_y)

def r3_autonomia(m, k): return sum(m.x[i, j, k] * m.d[i, j] for (i, j) in m.A) <= m.rng_k[k]
model.autonomia = Constraint(model.K, rule=r3_autonomia)

def r4_storage_base(m): return sum(m.q[j, k] * m.alpha for j in m.J for k in m.K) <= m.cap_i[I[0]]
model.storage_capacity = Constraint(rule=r4_storage_base)

def r5_serve_flow(m, j, k): return sum(m.x[i, j, k] for i in m.N if (i, j) in m.A) == m.y[j, k]
model.serve_flow = Constraint(model.J, model.K, rule=r5_serve_flow)

def r6_q_y_link(m, j, k): return m.q[j, k] == m.dem[j] * m.y[j, k]
model.q_y_link = Constraint(model.J, model.K, rule=r6_q_y_link)

def r7_flow_conservation(m, j, k):
    return sum(m.x[i, j, k] for i in m.N if (i, j) in m.A) == \
           sum(m.x[j, i, k] for i in m.N if (j, i) in m.A)
model.flujo = Constraint(model.J, model.K, rule=r7_flow_conservation)

def r8_salida_depot(m, i, k): return sum(m.x[i, j, k] for j in m.N if (i, j) in m.A) >= m.z[i, k]
model.salida_depot = Constraint(model.I, model.K, rule=r8_salida_depot)

def r9_subtours(m, i, j, k):
    if i != j:
        return m.u[i] - m.u[j] + len(m.J) * m.x[i, j, k] <= len(m.J) - 1
    return Constraint.Skip
model.subtours = Constraint(model.J, model.J, model.K, rule=r9_subtours)

# heuristica para warm-start
unserved = set(J)
initial_routes = {k: [] for k in K}
for k in K:
    route = [I[0]]  
    capacity_left = cap_k_dict[k]
    current = I[0]
    while True:
        
        candidates = [(j, d_data[(current, j)]) for j in unserved if dem_dict[j] <= capacity_left and (current, j) in d_data]
        if not candidates:
            break
        next_client, _ = min(candidates, key=lambda x: x[1])
        route.append(next_client)
        capacity_left -= dem_dict[next_client]
        unserved.remove(next_client)
        current = next_client
    route.append(I[0])  
    initial_routes[k] = route


for k, route in initial_routes.items():
    for i, j in zip(route, route[1:]):
        if (i, j) in model.A:
            model.x[i, j, k].value = 1
            
            if j in J:
                model.y[j, k].value = 1
                model.q[j, k].value = dem_dict[j]
    model.z[I[0], k].value = 1


solver = SolverFactory('glpk')
solver.options['tmlim'] = 100     
solver.solve(model, tee=True)

# mostrar rutas fin
for k in model.K:
    route = []
    for (i, j) in model.A:
        val = model.x[i, j, k].value
        if val is not None and val > 0.5:
            route.append((i, j))
    if route:
        print(f"Vehículo {k}: {route}")
    else:
        print(f"Vehículo {k}: no se asignaron arcos en la solución encontrada")

average_speed = 25  

with open('verificacion_caso1.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    
    writer.writerow([
        'VehicleId', 'DepotId', 'InitialLoad', 'RouteSequence',
        'ClientsServed', 'DemandsSatisfied',
        'TotalDistance', 'TotalTime', 'FuelCost'
    ])
    
    for k in model.K:
        
        arcs = [(i, j) for (i, j) in model.A 
                if model.x[i, j, k].value is not None and model.x[i, j, k].value > 0.5]
        if not arcs:
            continue
        
        seq = [arcs[0][0]]
        for (_, j) in arcs:
            seq.append(j)
        
        demands = [dem_dict[j] for (_, j) in arcs if j in dem_dict]
        initial_load = sum(demands)
        clients_served = len(demands)
        demands_satisfied_str = '-'.join(str(d) for d in demands)
        
        total_dist = sum(d_data[(i, j)] for (i, j) in arcs)
        total_time = total_dist / average_speed * 60
        fuel_cost = total_dist * model.Pf.value
        
        # Escribir fila
        writer.writerow([
            k,                  # VehicleId
            I[0],               # DepotId 
            initial_load,       # InitialLoad
            '-'.join(seq),      # RouteSequence
            clients_served,     # ClientsServed
            demands_satisfied_str,
            round(total_dist, 2),
            round(total_time, 1),
            int(fuel_cost)
        ])

print("verificacion_caso1.csv generado.")