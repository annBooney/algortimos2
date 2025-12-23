<?php

namespace DroneRouting\Metaheuristic;

use DroneRouting\Common\{Graph, Solution, Objectives, ParetoFront};

/**
 * Algoritmo NSGA-II para TSP multi-objetivo
 */
class NSGA2
{
    private Graph $graph;
    private int $populationSize;
    private int $maxGenerations;
    private float $crossoverRate;
    private float $mutationRate;

    public function __construct(
        Graph $graph,
        int $populationSize = 100,
        int $maxGenerations = 200,
        float $crossoverRate = 0.9,
        float $mutationRate = 0.1
    ) {
        $this->graph = $graph;
        $this->populationSize = $populationSize;
        $this->maxGenerations = $maxGenerations;
        $this->crossoverRate = $crossoverRate;
        $this->mutationRate = $mutationRate;
    }

    /**
     * Ejecuta NSGA-II y retorna el frente de Pareto
     */
    public function solve(): ParetoFront
    {
        // Inicializar población
        $population = $this->initializePopulation();

        for ($gen = 0; $gen < $this->maxGenerations; $gen++) {
            // Crear descendencia
            $offspring = $this->createOffspring($population);

            // Combinar población y descendencia
            $combined = array_merge($population, $offspring);

            // Ranking no dominado
            $fronts = $this->fastNonDominatedSort($combined);

            // Seleccionar nueva población
            $population = $this->selectNextGeneration($fronts);
        }

        // Extraer frente de Pareto
        $paretoFront = new ParetoFront();
        foreach ($population as $individual) {
            if ($individual->feasible && $individual->rank === 0) {
                $paretoFront->add(clone $individual);
            }
        }

        // Si el frente está vacío, añadir las mejores soluciones
        if ($paretoFront->size() === 0) {
            foreach ($population as $individual) {
                if ($individual->feasible) {
                    $paretoFront->add(clone $individual);
                }
            }
        }

        return $paretoFront;
    }

    /**
     * Inicializa la población con permutaciones aleatorias
     * @return Solution[]
     */
    private function initializePopulation(): array
    {
        $population = [];
        $n = $this->graph->nodeCount();

        for ($i = 0; $i < $this->populationSize; $i++) {
            $route = $this->generateRandomRoute($n);
            $solution = $this->graph->evaluateRoute($route);
            $population[] = $solution;
        }

        return $population;
    }

    /**
     * Genera una ruta aleatoria válida
     */
    private function generateRandomRoute(int $n): array
    {
        $nodes = range(1, $n - 1);
        shuffle($nodes);
        return array_merge([0], $nodes, [0]);
    }

    /**
     * Crea descendencia mediante selección, cruce y mutación
     * @return Solution[]
     */
    private function createOffspring(array $population): array
    {
        $offspring = [];

        // Asignar ranks y crowding distance
        $fronts = $this->fastNonDominatedSort($population);
        $this->assignCrowdingDistance($population);

        while (count($offspring) < $this->populationSize) {
            // Selección por torneo
            $parent1 = $this->tournamentSelection($population);
            $parent2 = $this->tournamentSelection($population);

            // Cruce
            if (mt_rand() / mt_getrandmax() < $this->crossoverRate) {
                [$child1Route, $child2Route] = $this->orderCrossover($parent1->route, $parent2->route);
            } else {
                $child1Route = $parent1->route;
                $child2Route = $parent2->route;
            }

            // Mutación
            if (mt_rand() / mt_getrandmax() < $this->mutationRate) {
                $child1Route = $this->mutate($child1Route);
            }
            if (mt_rand() / mt_getrandmax() < $this->mutationRate) {
                $child2Route = $this->mutate($child2Route);
            }

            // Reparar y evaluar
            $child1Route = $this->repairRoute($child1Route);
            $child2Route = $this->repairRoute($child2Route);

            $offspring[] = $this->graph->evaluateRoute($child1Route);
            if (count($offspring) < $this->populationSize) {
                $offspring[] = $this->graph->evaluateRoute($child2Route);
            }
        }

        return $offspring;
    }

    /**
     * Order Crossover (OX)
     */
    private function orderCrossover(array $parent1, array $parent2): array
    {
        // Remover el hub del inicio y final
        $p1 = array_slice($parent1, 1, -1);
        $p2 = array_slice($parent2, 1, -1);
        $n = count($p1);

        if ($n < 2) {
            return [$parent1, $parent2];
        }

        // Puntos de corte aleatorios
        $cut1 = mt_rand(0, $n - 2);
        $cut2 = mt_rand($cut1 + 1, $n - 1);

        // Crear hijos
        $child1 = $this->createOXChild($p1, $p2, $cut1, $cut2);
        $child2 = $this->createOXChild($p2, $p1, $cut1, $cut2);

        // Añadir hub
        return [
            array_merge([0], $child1, [0]),
            array_merge([0], $child2, [0])
        ];
    }

    private function createOXChild(array $p1, array $p2, int $cut1, int $cut2): array
    {
        $n = count($p1);
        $child = array_fill(0, $n, -1);

        // Copiar segmento del padre 1
        for ($i = $cut1; $i <= $cut2; $i++) {
            $child[$i] = $p1[$i];
        }

        // Elementos copiados
        $copied = array_filter($child, fn($x) => $x !== -1);

        // Llenar con padre 2
        $j = ($cut2 + 1) % $n;
        $p2Idx = $j;
        
        while (in_array(-1, $child)) {
            while (in_array($p2[$p2Idx], $copied)) {
                $p2Idx = ($p2Idx + 1) % $n;
            }
            
            while ($child[$j] !== -1) {
                $j = ($j + 1) % $n;
            }
            
            $child[$j] = $p2[$p2Idx];
            $copied[] = $p2[$p2Idx];
            $p2Idx = ($p2Idx + 1) % $n;
        }

        return $child;
    }

    /**
     * Mutación: swap, inversión o inserción
     */
    private function mutate(array $route): array
    {
        $type = mt_rand(0, 2);
        $inner = array_slice($route, 1, -1);
        $n = count($inner);

        if ($n < 2) return $route;

        switch ($type) {
            case 0: // Swap
                $i = mt_rand(0, $n - 1);
                $j = mt_rand(0, $n - 1);
                $temp = $inner[$i];
                $inner[$i] = $inner[$j];
                $inner[$j] = $temp;
                break;

            case 1: // Inversión
                $i = mt_rand(0, $n - 2);
                $j = mt_rand($i + 1, $n - 1);
                $segment = array_reverse(array_slice($inner, $i, $j - $i + 1));
                array_splice($inner, $i, $j - $i + 1, $segment);
                break;

            case 2: // Inserción
                $i = mt_rand(0, $n - 1);
                $j = mt_rand(0, $n - 1);
                if ($i !== $j) {
                    $elem = $inner[$i];
                    array_splice($inner, $i, 1);
                    array_splice($inner, $j, 0, [$elem]);
                }
                break;
        }

        return array_merge([0], $inner, [0]);
    }

    /**
     * Repara una ruta para asegurar que sea una permutación válida
     */
    private function repairRoute(array $route): array
    {
        $n = $this->graph->nodeCount();
        $inner = array_slice($route, 1, -1);
        
        // Asegurar que contenga todos los nodos 1..n-1
        $required = range(1, $n - 1);
        $present = array_intersect($inner, $required);
        $missing = array_diff($required, $present);
        
        // Encontrar duplicados
        $seen = [];
        $duplicateIdx = [];
        foreach ($inner as $idx => $val) {
            if (in_array($val, $seen) || !in_array($val, $required)) {
                $duplicateIdx[] = $idx;
            } else {
                $seen[] = $val;
            }
        }
        
        // Reemplazar duplicados con faltantes
        $missing = array_values($missing);
        foreach ($duplicateIdx as $i => $idx) {
            if (isset($missing[$i])) {
                $inner[$idx] = $missing[$i];
            }
        }
        
        // Si aún faltan nodos, añadirlos
        $present = $inner;
        $missing = array_diff($required, $present);
        foreach ($missing as $node) {
            $inner[] = $node;
        }
        
        // Asegurar longitud correcta
        $inner = array_slice($inner, 0, $n - 1);
        
        return array_merge([0], $inner, [0]);
    }

    /**
     * Selección por torneo binario
     */
    private function tournamentSelection(array $population): Solution
    {
        $i1 = mt_rand(0, count($population) - 1);
        $i2 = mt_rand(0, count($population) - 1);
        
        $ind1 = $population[$i1];
        $ind2 = $population[$i2];

        // Preferir menor rango
        if ($ind1->rank < $ind2->rank) return $ind1;
        if ($ind2->rank < $ind1->rank) return $ind2;

        // Si mismo rango, preferir mayor crowding distance
        return $ind1->crowdingDistance >= $ind2->crowdingDistance ? $ind1 : $ind2;
    }

    /**
     * Fast Non-dominated Sorting (NSGA-II)
     * @return Solution[][] Fronts ordenados
     */
    private function fastNonDominatedSort(array $population): array
    {
        $n = count($population);
        $dominatedBy = array_fill(0, $n, []); // Quién domina a cada individuo
        $dominates = array_fill(0, $n, []);   // A quién domina cada individuo
        $dominationCount = array_fill(0, $n, 0);

        // Calcular dominancias
        for ($i = 0; $i < $n; $i++) {
            for ($j = $i + 1; $j < $n; $j++) {
                if ($population[$i]->dominates($population[$j])) {
                    $dominates[$i][] = $j;
                    $dominationCount[$j]++;
                } elseif ($population[$j]->dominates($population[$i])) {
                    $dominates[$j][] = $i;
                    $dominationCount[$i]++;
                }
            }
        }

        // Construir fronts
        $fronts = [];
        $currentFront = [];

        for ($i = 0; $i < $n; $i++) {
            if ($dominationCount[$i] === 0) {
                $population[$i]->rank = 0;
                $currentFront[] = $i;
            }
        }

        $frontIdx = 0;
        while (!empty($currentFront)) {
            $fronts[$frontIdx] = array_map(fn($i) => $population[$i], $currentFront);
            $nextFront = [];

            foreach ($currentFront as $i) {
                foreach ($dominates[$i] as $j) {
                    $dominationCount[$j]--;
                    if ($dominationCount[$j] === 0) {
                        $population[$j]->rank = $frontIdx + 1;
                        $nextFront[] = $j;
                    }
                }
            }

            $currentFront = $nextFront;
            $frontIdx++;
        }

        return $fronts;
    }

    /**
     * Asigna crowding distance a cada individuo
     */
    private function assignCrowdingDistance(array &$population): void
    {
        foreach ($population as $ind) {
            $ind->crowdingDistance = 0;
        }

        // Agrupar por rank
        $byRank = [];
        foreach ($population as $ind) {
            $byRank[$ind->rank][] = $ind;
        }

        foreach ($byRank as $front) {
            $this->calculateCrowdingDistance($front);
        }
    }

    /**
     * Calcula crowding distance para un frente
     */
    private function calculateCrowdingDistance(array &$front): void
    {
        $n = count($front);
        if ($n <= 2) {
            foreach ($front as $ind) {
                $ind->crowdingDistance = PHP_FLOAT_MAX;
            }
            return;
        }

        foreach ($front as $ind) {
            $ind->crowdingDistance = 0;
        }

        // Para cada objetivo
        $objectives = ['distance', 'risk', 'recharges'];
        
        foreach ($objectives as $obj) {
            // Ordenar por este objetivo
            usort($front, function($a, $b) use ($obj) {
                return $a->objectives->$obj <=> $b->objectives->$obj;
            });

            // Extremos tienen distancia infinita
            $front[0]->crowdingDistance = PHP_FLOAT_MAX;
            $front[$n - 1]->crowdingDistance = PHP_FLOAT_MAX;

            // Rango del objetivo
            $range = $front[$n - 1]->objectives->$obj - $front[0]->objectives->$obj;
            if ($range == 0) continue;

            // Calcular distancias
            for ($i = 1; $i < $n - 1; $i++) {
                $front[$i]->crowdingDistance += 
                    ($front[$i + 1]->objectives->$obj - $front[$i - 1]->objectives->$obj) / $range;
            }
        }
    }

    /**
     * Selecciona la siguiente generación
     * @return Solution[]
     */
    private function selectNextGeneration(array $fronts): array
    {
        $newPop = [];
        
        foreach ($fronts as $front) {
            if (count($newPop) + count($front) <= $this->populationSize) {
                $newPop = array_merge($newPop, $front);
            } else {
                // Necesitamos solo parte de este frente
                $this->calculateCrowdingDistance($front);
                usort($front, fn($a, $b) => $b->crowdingDistance <=> $a->crowdingDistance);
                
                $remaining = $this->populationSize - count($newPop);
                $newPop = array_merge($newPop, array_slice($front, 0, $remaining));
                break;
            }
        }

        return $newPop;
    }
}
