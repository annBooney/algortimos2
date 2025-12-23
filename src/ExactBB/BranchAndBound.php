<?php

namespace DroneRouting\ExactBB;

use DroneRouting\Common\{Graph, Solution, Objectives, ParetoFront, Node};

/**
 * Algoritmo Branch & Bound para TSP multi-objetivo
 */
class BranchAndBound
{
    private Graph $graph;
    private ParetoFront $paretoFront;
    private int $nodesExplored;
    private int $nodesPruned;
    private ?int $maxNodes;

    public function __construct(Graph $graph, ?int $maxNodes = null)
    {
        $this->graph = $graph;
        $this->paretoFront = new ParetoFront();
        $this->nodesExplored = 0;
        $this->nodesPruned = 0;
        $this->maxNodes = $maxNodes;
    }

    /**
     * Resuelve el problema y retorna el frente de Pareto
     */
    public function solve(): ParetoFront
    {
        $n = $this->graph->nodeCount();
        
        // Comenzar desde el hub (nodo 0)
        $initialPath = [0];
        $visited = array_fill(0, $n, false);
        $visited[0] = true;
        $initialObjectives = new Objectives(0, 0, 0);
        $initialBattery = 100;

        $this->branchAndBound($initialPath, $visited, $initialObjectives, $initialBattery);

        return $this->paretoFront;
    }

    /**
     * Búsqueda recursiva con podas
     */
    private function branchAndBound(
        array $path,
        array $visited,
        Objectives $currentObj,
        float $battery
    ): void {
        $this->nodesExplored++;

        // Verificar límite de nodos
        if ($this->maxNodes !== null && $this->nodesExplored > $this->maxNodes) {
            return;
        }

        $n = $this->graph->nodeCount();
        $current = end($path);

        // Si visitamos todos los nodos, cerrar el circuito
        if (count($path) === $n) {
            $edge = $this->graph->getEdge($current, 0);
            if ($edge && $edge->valid) {
                $finalObj = new Objectives(
                    $currentObj->distance + $edge->weight->distance,
                    $currentObj->risk + $edge->weight->risk,
                    $currentObj->recharges
                );

                // Verificar batería para el regreso
                $finalBattery = $battery - $edge->weight->battery;
                if ($finalBattery < 0) {
                    $finalObj->recharges++;
                }

                $solution = new Solution(array_merge($path, [0]), $finalObj, true);
                $this->paretoFront->add($solution);
            }
            return;
        }

        // Calcular cota inferior
        $lowerBound = $this->calculateLowerBound($path, $visited, $currentObj);

        // Poda por dominancia: si la cota inferior está dominada, podar
        if ($this->isDominatedByFront($lowerBound)) {
            $this->nodesPruned++;
            return;
        }

        // Expandir a nodos no visitados
        $unvisited = [];
        for ($i = 0; $i < $n; $i++) {
            if (!$visited[$i]) {
                $edge = $this->graph->getEdge($current, $i);
                if ($edge && $edge->valid) {
                    $unvisited[] = $i;
                }
            }
        }

        // Ordenar por distancia (heurística de ordenamiento)
        usort($unvisited, function($a, $b) use ($current) {
            $edgeA = $this->graph->getEdge($current, $a);
            $edgeB = $this->graph->getEdge($current, $b);
            return $edgeA->weight->distance <=> $edgeB->weight->distance;
        });

        foreach ($unvisited as $next) {
            $edge = $this->graph->getEdge($current, $next);
            
            // Actualizar objetivos
            $newObj = new Objectives(
                $currentObj->distance + $edge->weight->distance,
                $currentObj->risk + $edge->weight->risk,
                $currentObj->recharges
            );

            // Simular batería
            $newBattery = $battery - $edge->weight->battery;
            if ($newBattery < 20) {
                $newObj->recharges++;
                $newBattery = 100;
            }

            // Recarga en estación
            $nextNode = $this->graph->getNode($next);
            if ($nextNode->isCharging && $newBattery < 80) {
                $newBattery = 100;
            }

            // Actualizar camino y visitados
            $newPath = array_merge($path, [$next]);
            $newVisited = $visited;
            $newVisited[$next] = true;

            $this->branchAndBound($newPath, $newVisited, $newObj, $newBattery);
        }
    }

    /**
     * Calcula cota inferior usando MST sobre nodos no visitados
     */
    private function calculateLowerBound(array $path, array $visited, Objectives $currentObj): Objectives
    {
        $n = $this->graph->nodeCount();
        $current = end($path);

        // Nodos no visitados + hub (para cerrar el circuito)
        $remaining = [0]; // hub
        for ($i = 1; $i < $n; $i++) {
            if (!$visited[$i]) {
                $remaining[] = $i;
            }
        }

        if (count($remaining) <= 1) {
            // Solo falta volver al hub
            $edge = $this->graph->getEdge($current, 0);
            if ($edge && $edge->valid) {
                return new Objectives(
                    $currentObj->distance + $edge->weight->distance,
                    $currentObj->risk + $edge->weight->risk,
                    $currentObj->recharges
                );
            }
            return $currentObj;
        }

        // Calcular MST sobre remaining + current
        $nodes = array_merge([$current], $remaining);
        $mstCost = $this->calculateMST($nodes);

        return new Objectives(
            $currentObj->distance + $mstCost['distance'],
            $currentObj->risk + $mstCost['risk'],
            $currentObj->recharges
        );
    }

    /**
     * Calcula el MST usando Prim
     */
    private function calculateMST(array $nodes): array
    {
        $n = count($nodes);
        if ($n <= 1) {
            return ['distance' => 0, 'risk' => 0];
        }

        $inMST = array_fill(0, $n, false);
        $key = array_fill(0, $n, PHP_FLOAT_MAX);
        $keyRisk = array_fill(0, $n, 0);
        $key[0] = 0;

        for ($count = 0; $count < $n - 1; $count++) {
            // Encontrar el mínimo
            $minKey = PHP_FLOAT_MAX;
            $u = -1;
            for ($i = 0; $i < $n; $i++) {
                if (!$inMST[$i] && $key[$i] < $minKey) {
                    $minKey = $key[$i];
                    $u = $i;
                }
            }

            if ($u === -1) break;
            $inMST[$u] = true;

            // Actualizar claves
            for ($v = 0; $v < $n; $v++) {
                if (!$inMST[$v]) {
                    $edge = $this->graph->getEdge($nodes[$u], $nodes[$v]);
                    if ($edge && $edge->valid && $edge->weight->distance < $key[$v]) {
                        $key[$v] = $edge->weight->distance;
                        $keyRisk[$v] = $edge->weight->risk;
                    }
                }
            }
        }

        $totalDist = 0;
        $totalRisk = 0;
        for ($i = 1; $i < $n; $i++) {
            if ($key[$i] < PHP_FLOAT_MAX) {
                $totalDist += $key[$i];
                $totalRisk += $keyRisk[$i];
            }
        }

        return ['distance' => $totalDist, 'risk' => $totalRisk];
    }

    /**
     * Verifica si unos objetivos están dominados por alguna solución del frente
     */
    private function isDominatedByFront(Objectives $obj): bool
    {
        foreach ($this->paretoFront->getSolutions() as $solution) {
            if ($solution->objectives->dominates($obj)) {
                return true;
            }
        }
        return false;
    }

    public function getStats(): array
    {
        return [
            'nodes_explored' => $this->nodesExplored,
            'nodes_pruned' => $this->nodesPruned,
            'pareto_size' => $this->paretoFront->size()
        ];
    }
}
