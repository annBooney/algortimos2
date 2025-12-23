# 🚁 Optimización Multi-objetivo de Rutas de Drones (PHP)

Sistema de planificación de rutas para drones de reparto que resuelve una variante multi-objetivo del Problema del Viajante (TSP) con restricciones de zonas de vuelo prohibido.

## 📋 Descripción del Problema

Dado un mapa con N vértices (puntos de entrega y estaciones de recarga) y zonas no-fly representadas como polígonos, se busca encontrar el circuito hamiltoniano óptimo que:

- **Minimiza** la distancia total recorrida
- **Minimiza** el riesgo acumulado (proximidad a zonas prohibidas)
- **Minimiza** el número de paradas de recarga
- **Evita** intersecciones con zonas no-fly

## 🏗️ Estructura del Proyecto

```
drone_routing_php/
├── src/
│   ├── Common/
│   │   ├── Geometry.php          # Geometría computacional
│   │   ├── DataStructures.php    # Graph, Node, Edge, Solution, ParetoFront
│   │   └── Utils.php             # Generación de instancias y utilidades
│   ├── ExactBB/
│   │   └── BranchAndBound.php    # Método exacto con poda Pareto
│   ├── GeoHeuristic/
│   │   └── GeometricHeuristics.php # NN, Insertion, Sweep, ConvexHull
│   └── Metaheuristic/
│       └── NSGA2.php             # Algoritmo evolutivo multi-objetivo
├── instances/                     # Instancias de prueba (.json)
├── results/                       # Resultados experimentales
├── main.php                       # Interfaz CLI
├── Makefile                       # Automatización
└── README.md
```

## Requisitos

- PHP 8.0 o superior
- Sin dependencias externas (solo biblioteca estándar)

## Instalación y Ejecución

### Ejecución Completa
```bash
make run
```

Este comando:
1. Genera 4 instancias de prueba (N=10, 15, 20, 25)
2. Ejecuta los tres algoritmos con 5 réplicas cada uno
3. Muestra tabla resumen de resultados
4. Guarda resultados en `results/experiment_results.json`

### Comandos Individuales

```bash
# Generar instancia personalizada
php main.php generate --nodes 20 --zones 4 --seed 42 --output instance.json

# Resolver con algoritmo específico
php main.php solve instances/instance_n15.json --algorithm exact
php main.php solve instances/instance_n15.json --algorithm heuristic
php main.php solve instances/instance_n15.json --algorithm meta
php main.php solve instances/instance_n15.json --algorithm all

# Ejecutar experimentos
php main.php experiment --replicas 5 --max-bb 15

# Prueba rápida
make test
```