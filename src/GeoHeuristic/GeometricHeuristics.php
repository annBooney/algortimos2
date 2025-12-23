<?php

namespace DroneRouting\GeoHeuristic;

use DroneRouting\Common\{Graph, Solution, Objectives, ParetoFront, Point, Geometry};

/**
 * Heurísticas geométricas para TSP multi-objetivo
 */
class GeometricHeuristics
{
    private Graph $graph;

    public function __construct(Graph $graph)
    {
        $this->graph = $graph;
    }

    /**
     * Ejecuta todas las heurísticas y combina resultados
     */
    public function solve(): ParetoFront
    {
        $front = new ParetoFront();

        // Vecino más cercano con diferentes pesos
        $nnSolutions = $this->nearestNeighborAll();
        foreach ($nnSolutions as $sol) {
            $front->add($sol);
        }

        // Inserción más barata
        $insSolutions = $this->cheapestInsertionAll();
        foreach ($insSolutions as $sol) {
            $front->add($sol);
        }

        // Heurística de barrido
        $sweepSolutions = $this->sweepAll();
        foreach ($sweepSolutions as $sol) {
            $front->add($sol);
        }

        // Inserción en envolvente convexa
        $chSolution = $this->convexHullInsertion();
        if ($chSolution) {
            $front->add($chSolution);
        }

        // Aplicar 2-opt a todas las soluciones
        $improved = new ParetoFront();
        foreach ($front->getSolutions() as $sol) {
            $improvedSol = $this->twoOpt($sol);
            $improved->add($improvedSol);
        }

        return $improved;
    }

    /**
     * Vecino más cercano con múltiples combinaciones de pesos
     * @return Solution[]
     */
    public function nearestNeighborAll(): array
    {
        $solutions = [];
        
        // Combinaciones de pesos: (w_dist, w_risk)
        $weights = [];
        for ($d = 0; $d <= 10; $d += 2) {
            for ($r = 0; $r <= 10 - $d; $r += 2) {
                $weights[] = [$d / 10, $r / 10];
            }
        }

        foreach ($weights as $w) {
            $sol = $this->nearestNeighbor($w[0], $w[1]);
            if ($sol && $sol->feasible) {
                $solutions[] = $sol;
            }
        }

        return $solutions;
    }

    /**
     * Vecino más cercano con pesos específicos
     */
    public function nearestNeighbor(float $wDist = 1.0, float $wRisk = 0.0): ?Solution
    {
        $n = $this->graph->nodeCount();
        $visited = array_fill(0, $n, false);
        $path = [0];
        $visited[0] = true;

        while (count($path) < $n) {
            $current = end($path);
            $best = -1;
            $bestScore = PHP_FLOAT_MAX;

            for ($i = 0; $i < $n; $i++) {
                if (!$visited[$i]) {
                    $edge = $this->graph->getEdge($current, $i);
                    if ($edge && $edge->valid) {
                        $score = $wDist * $edge->weight->distance + $wRisk * $edge->weight->risk;
                        if ($score < $bestScore) {
                            $bestScore = $score;
                            $best = $i;
                        }
                    }
                }
            }

            if ($best === -1) {
                // No hay arista válida, buscar cualquiera
                for ($i = 0; $i < $n; $i++) {
                    if (!$visited[$i]) {
                        $best = $i;
                        break;
                    }
                }
            }

            if ($best === -1) break;

            $path[] = $best;
            $visited[$best] = true;
        }

        // Cerrar circuito
        $path[] = 0;
        return $this->graph->evaluateRoute($path);
    }

    /**
     * Inserción más barata con múltiples pesos
     * @return Solution[]
     */
    public function cheapestInsertionAll(): array
    {
        $solutions = [];
        
        $weights = [];
        for ($d = 0; $d <= 10; $d += 2) {
            $weights[] = [$d / 10, (10 - $d) / 10];
        }

        foreach ($weights as $w) {
            $sol = $this->cheapestInsertion($w[0], $w[1]);
            if ($sol && $sol->feasible) {
                $solutions[] = $sol;
            }
        }

        return $solutions;
    }

    /**
     * Inserción más barata
     */
    public function cheapestInsertion(float $wDist = 1.0, float $wRisk = 0.0): ?Solution
    {
        $n = $this->graph->nodeCount();
        
        // Comenzar con un triángulo: 0 -> nodo más lejano -> nodo más lejano -> 0
        $farthest = $this->findFarthestNode(0);
        if ($farthest === -1) return null;
        
        $second = $this->findFarthestNodeExcluding(0, [$farthest]);
        if ($second === -1) $second = $farthest === 1 ? 2 : 1;

        $tour = [0, $farthest, $second]; // Sin cerrar aún
        $inTour = array_fill(0, $n, false);
        $inTour[0] = $inTour[$farthest] = $inTour[$second] = true;

        while (count($tour) < $n) {
            $bestNode = -1;
            $bestPos = -1;
            $bestCost = PHP_FLOAT_MAX;

            // Para cada nodo no en el tour
            for ($node = 0; $node < $n; $node++) {
                if ($inTour[$node]) continue;

                // Para cada posición de inserción
                for ($pos = 1; $pos <= count($tour); $pos++) {
                    $prev = $tour[$pos - 1];
                    $next = $tour[$pos % count($tour)];

                    // Siguiente en el tour (cerrado)
                    if ($pos === count($tour)) {
                        $next = $tour[0];
                    }

                    $edgePrev = $this->graph->getEdge($prev, $node);
                    $edgeNext = $this->graph->getEdge($node, $next);
                    $edgeCurrent = $this->graph->getEdge($prev, $next);

                    if (!$edgePrev || !$edgeNext || !$edgeCurrent) continue;
                    if (!$edgePrev->valid || !$edgeNext->valid) continue;

                    $costIncrease = ($edgePrev->weight->distance + $edgeNext->weight->distance - $edgeCurrent->weight->distance) * $wDist +
                                   ($edgePrev->weight->risk + $edgeNext->weight->risk - $edgeCurrent->weight->risk) * $wRisk;

                    if ($costIncrease < $bestCost) {
                        $bestCost = $costIncrease;
                        $bestNode = $node;
                        $bestPos = $pos;
                    }
                }
            }

            if ($bestNode === -1) {
                // Insertar cualquier nodo restante
                for ($node = 0; $node < $n; $node++) {
                    if (!$inTour[$node]) {
                        $bestNode = $node;
                        $bestPos = count($tour);
                        break;
                    }
                }
            }

            if ($bestNode === -1) break;

            array_splice($tour, $bestPos, 0, [$bestNode]);
            $inTour[$bestNode] = true;
        }

        // Cerrar circuito
        $tour[] = $tour[0];
        return $this->graph->evaluateRoute($tour);
    }

    /**
     * Heurística de barrido con múltiples ángulos iniciales
     * @return Solution[]
     */
    public function sweepAll(): array
    {
        $solutions = [];
        
        // Diferentes ángulos iniciales (0, 30, 60, ..., 330 grados)
        for ($angle = 0; $angle < 360; $angle += 30) {
            $sol = $this->sweep($angle * M_PI / 180, true);
            if ($sol && $sol->feasible) {
                $solutions[] = $sol;
            }

            $sol = $this->sweep($angle * M_PI / 180, false);
            if ($sol && $sol->feasible) {
                $solutions[] = $sol;
            }
        }

        return $solutions;
    }

    /**
     * Heurística de barrido angular
     */
    public function sweep(float $startAngle = 0, bool $clockwise = true): ?Solution
    {
        $n = $this->graph->nodeCount();
        $hub = $this->graph->getNode(0);
        $hubPos = $hub->position;

        // Calcular ángulos polares de todos los nodos
        $angles = [];
        for ($i = 1; $i < $n; $i++) {
            $node = $this->graph->getNode($i);
            $angle = Geometry::polarAngle($hubPos, $node->position);
            // Normalizar respecto al ángulo inicial
            $angle = fmod($angle - $startAngle + 2 * M_PI, 2 * M_PI);
            $angles[$i] = $angle;
        }

        // Ordenar por ángulo
        if ($clockwise) {
            asort($angles);
        } else {
            arsort($angles);
        }

        $tour = [0];
        foreach (array_keys($angles) as $nodeId) {
            $tour[] = $nodeId;
        }
        $tour[] = 0;

        return $this->graph->evaluateRoute($tour);
    }

    /**
     * Inserción en envolvente convexa
     */
    public function convexHullInsertion(): ?Solution
    {
        $n = $this->graph->nodeCount();
        
        // Obtener posiciones
        $points = [];
        $pointToNode = [];
        for ($i = 0; $i < $n; $i++) {
            $node = $this->graph->getNode($i);
            $points[] = $node->position;
            $pointToNode[spl_object_id($node->position)] = $i;
        }

        // Calcular envolvente convexa
        $hull = Geometry::convexHull($points);
        
        if (count($hull) < 3) {
            return $this->nearestNeighbor();
        }

        // Construir tour inicial con el hull
        $tour = [];
        $inTour = array_fill(0, $n, false);
        
        foreach ($hull as $point) {
            // Encontrar el nodo correspondiente
            for ($i = 0; $i < $n; $i++) {
                $node = $this->graph->getNode($i);
                if (abs($node->position->x - $point->x) < 0.001 && 
                    abs($node->position->y - $point->y) < 0.001) {
                    if (!$inTour[$i]) {
                        $tour[] = $i;
                        $inTour[$i] = true;
                    }
                    break;
                }
            }
        }

        // Asegurarse de que el hub esté en el tour
        if (!$inTour[0]) {
            array_unshift($tour, 0);
            $inTour[0] = true;
        }

        // Insertar nodos interiores
        for ($node = 0; $node < $n; $node++) {
            if ($inTour[$node]) continue;

            $bestPos = 1;
            $bestCost = PHP_FLOAT_MAX;

            for ($pos = 1; $pos <= count($tour); $pos++) {
                $prev = $tour[$pos - 1];
                $next = $tour[$pos % count($tour)];

                $edgePrev = $this->graph->getEdge($prev, $node);
                $edgeNext = $this->graph->getEdge($node, $next);
                $edgeCurrent = $this->graph->getEdge($prev, $next);

                if (!$edgePrev || !$edgeNext || !$edgeCurrent) continue;
                if (!$edgePrev->valid || !$edgeNext->valid) continue;

                $cost = $edgePrev->weight->distance + $edgeNext->weight->distance - $edgeCurrent->weight->distance;
                if ($cost < $bestCost) {
                    $bestCost = $cost;
                    $bestPos = $pos;
                }
            }

            array_splice($tour, $bestPos, 0, [$node]);
            $inTour[$node] = true;
        }

        // Rotar para que empiece en el hub
        $hubIndex = array_search(0, $tour);
        if ($hubIndex !== 0) {
            $tour = array_merge(array_slice($tour, $hubIndex), array_slice($tour, 0, $hubIndex));
        }

        $tour[] = 0;
        return $this->graph->evaluateRoute($tour);
    }

    /**
     * Mejora local 2-opt
     */
    public function twoOpt(Solution $solution): Solution
    {
        $route = $solution->route;
        $n = count($route) - 1; // Sin contar el retorno al inicio
        $improved = true;

        while ($improved) {
            $improved = false;

            for ($i = 1; $i < $n - 1; $i++) {
                for ($j = $i + 1; $j < $n; $j++) {
                    // Verificar si el swap es válido
                    $newRoute = $this->twoOptSwap($route, $i, $j);
                    
                    if ($this->isRouteValid($newRoute)) {
                        $newSolution = $this->graph->evaluateRoute($newRoute);
                        
                        // Aceptar si mejora en distancia y no empeora mucho en riesgo
                        if ($newSolution->objectives->distance < $solution->objectives->distance - 0.01) {
                            $route = $newRoute;
                            $solution = $newSolution;
                            $improved = true;
                            break 2;
                        }
                    }
                }
            }
        }

        return $solution;
    }

    /**
     * Realiza el swap 2-opt
     */
    private function twoOptSwap(array $route, int $i, int $j): array
    {
        $newRoute = array_slice($route, 0, $i);
        $newRoute = array_merge($newRoute, array_reverse(array_slice($route, $i, $j - $i + 1)));
        $newRoute = array_merge($newRoute, array_slice($route, $j + 1));
        return $newRoute;
    }

    /**
     * Verifica si una ruta es válida (todas las aristas existen y son válidas)
     */
    private function isRouteValid(array $route): bool
    {
        for ($i = 0; $i < count($route) - 1; $i++) {
            $edge = $this->graph->getEdge($route[$i], $route[$i + 1]);
            if (!$edge || !$edge->valid) {
                return false;
            }
        }
        return true;
    }

    private function findFarthestNode(int $from): int
    {
        $n = $this->graph->nodeCount();
        $farthest = -1;
        $maxDist = 0;

        for ($i = 0; $i < $n; $i++) {
            if ($i === $from) continue;
            $edge = $this->graph->getEdge($from, $i);
            if ($edge && $edge->valid && $edge->weight->distance > $maxDist) {
                $maxDist = $edge->weight->distance;
                $farthest = $i;
            }
        }

        return $farthest;
    }

    private function findFarthestNodeExcluding(int $from, array $exclude): int
    {
        $n = $this->graph->nodeCount();
        $farthest = -1;
        $maxDist = 0;

        for ($i = 0; $i < $n; $i++) {
            if ($i === $from || in_array($i, $exclude)) continue;
            $edge = $this->graph->getEdge($from, $i);
            if ($edge && $edge->valid && $edge->weight->distance > $maxDist) {
                $maxDist = $edge->weight->distance;
                $farthest = $i;
            }
        }

        return $farthest;
    }
}
