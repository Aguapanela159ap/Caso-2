# README – Caso 1: Optimización en la Planeación de Transporte Vehicular Urbana

## 1. Descripción General

Este README describe cómo ejecutar el **Caso 1 (CVRP Estándar)** para la asignación de rutas de vehículos en Bogotá, implementado en Pyomo con GLPK como solver. El objetivo es minimizar el costo total de operación (combinando tarifa de flete, mantenimiento y combustible) satisfaciendo todas las demandas de los clientes.

## 2. Requisitos

* Python 3.7 o superior
* [Pyomo](https://pyomo.org/) (v6.x)
* Solver GLPK (instalado en el sistema, accesible como `glpsol`)
* Paquetes de Python: `csv`, `math`, `itertools`, `os`


## 3. Estructura de Directorios

```text
project_caso1/
├── clientsC1.csv       # Datos de clientes (ID, ubicación, demanda)
├── depotsC1.csv        # Datos de depósitos (ID, ubicación, capacidad)
├── vehiclesC1.csv      # Datos de vehículos (ID, capacidad, autonomía)
├── main.py           # Script principal Pyomo
└── verificacion_caso1.csv  # Salida generada por el script
```

## 4. Formato de Archivos de Entrada

* **clients.csv**: columnas `ClientID,Longitude,Latitude,Demand`.
* **depots.csv**: columnas `DepotID,Longitude,Latitude` (capacidad fija en el código).
* **vehicles.csv**: columnas `VehicleID,Capacity,Range`.

## 5. Uso

1. Colocar los tres CSV en el mismo directorio que `main.py`.
2. Ejecutar:

   ```bash
   python main.py
   ```
3. El script imprimirá en pantalla las rutas asignadas por vehículo y generará `verificacion_caso1.csv`.

## 6. Descripción del Proceso

1. **Lectura de datos**: carga de clientes, depósitos y vehículos.
2. **Cálculo de distancias**: matriz Haversine **(posible cambio en el futuro)** entre todos los nodos.
3. **Construcción del modelo**: definición de variables (`x,q,z,y,u`), parámetros y restricciones:

   * Satisfacción de demanda
   * Capacidad de vehículos
   * Autonomía por vehículo
   * Eliminación de subtours (MTZ)
4. **Heurística Nearest‑Neighbor**: warm‑start para mejorar la búsqueda.
5. **Optimización**: resolución con GLPK y límite de tiempo.
6. **Extracción de rutas**: reconstrucción de la secuencia de cada vehículo.
7. **Verificación**: cálculo de distancia, tiempo y costo de combustible, y escritura del CSV.

## 7. Salida (`verificacion_caso1.csv`)

| VehicleId | DepotId | InitialLoad | RouteSequence     | ClientsServed | DemandsSatisfied | TotalDistance | TotalTime | FuelCost |
| --------- | ------- | ----------- | ----------------- | ------------- | ---------------- | ------------- | --------- | -------- |
| VEH001    | CDA     | 750         | CDA-C005-C023-... | 3             | 215-320-215      | 28.4          | 67.2      | 98500    |

* **VehicleId**: identificador del vehículo.
* **DepotId**: depósito de origen.
* **InitialLoad**: suma de las demandas servidas.
* **RouteSequence**: nodos visitados separados por `-`.
* **ClientsServed**: número de clientes atendidos.
* **DemandsSatisfied**: lista de demandas por cliente.
* **TotalDistance** (km), **TotalTime** (min), **FuelCost** (COP).

## 8. Consideraciones

* La capacidad de los depósitos está fija en 1000 kg.
* Se asume velocidad promedio de 25 km/h para tiempo estimado.


---

