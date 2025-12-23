<?php

namespace DroneRouting\Common;

/**
 * Peso vectorial de una arista: <distancia, riesgo, batería>
 */
class EdgeWeight
{
    public float $distance;
    public float $risk;
    public float $battery;

    public function __construct(float $distance, float $risk, float $battery)
    {
        $this->distance = $distance;
        $this->risk = $risk;
        $this->battery = $battery;
    }

    /**
     * Verifica si este peso domina a otro (menor o igual en todo, estrictamente menor en al menos uno)
     */
    public function dominates(EdgeWeight $other): bool
    {
        $dominated = ($this->distance <= $other->distance) &&
                     ($this->risk <= $other->risk) &&
                     ($this->battery <= $other->battery);
        
        $strictlyBetter = ($this->distance < $other->distance) ||
                          ($this->risk < $other->risk) ||
                          ($this->battery < $other->battery);
        
        return $dominated && $strictlyBetter;
    }

    public function toArray(): array
    {
        return [
            'distance' => $this->distance,
            'risk' => $this->risk,
            'battery' => $this->battery
        ];
    }
}

/**
 * Nodo del grafo (punto de entrega o estación de recarga)
 */
class Node
{
    public int $id;
    public Point $position;
    public bool $isHub;
    public bool $isCharging;

    public function __construct(int $id, Point $position, bool $isHub = false, bool $isCharging = false)
    {
        $this->id = $id;
        $this->position = $position;
        $this->isHub = $isHub;
        $this->isCharging = $isCharging;
    }

    public function toArray(): array
    {
        return [
            'id' => $this->id,
            'x' => $this->position->x,
            'y' => $this->position->y,
            'is_hub' => $this->isHub,
            'is_charging' => $this->isCharging
        ];
    }

    public static function fromArray(array $arr): Node
    {
        return new Node(
            $arr['id'],
            new Point($arr['x'], $arr['y']),
            $arr['is_hub'] ?? false,
            $arr['is_charging'] ?? false
        );
    }
}

/**
 * Arista del grafo con peso vectorial
 */
class Edge
{
    public int $source;
    public int $target;
    public EdgeWeight $weight;
    public bool $valid;

    public function __construct(int $source, int $target, EdgeWeight $weight, bool $valid = true)
    {
        $this->source = $source;
        $this->target = $target;
        $this->weight = $weight;
        $this->valid = $valid;
    }
}

/**
 * Objetivos de una solución
 */
class Objectives
{
    public float $distance;
    public float $risk;
    public int $recharges;

    public function __construct(float $distance = 0, float $risk = 0, int $recharges = 0)
    {
        $this->distance = $distance;
        $this->risk = $risk;
        $this->recharges = $recharges;
    }

    /**
     * Verifica si estos objetivos dominan a otros
     */
    public function dominates(Objectives $other): bool
    {
        $dominated = ($this->distance <= $other->distance) &&
                     ($this->risk <= $other->risk) &&
                     ($this->recharges <= $other->recharges);
        
        $strictlyBetter = ($this->distance < $other->distance) ||
                          ($this->risk < $other->risk) ||
                          ($this->recharges < $other->recharges);
        
        return $dominated && $strictlyBetter;
    }

    public function toArray(): array
    {
        return [
            'distance' => round($this->distance, 4),
            'risk' => round($this->risk, 4),
            'recharges' => $this->recharges
        ];
    }

    public function toVector(): array
    {
        return [$this->distance, $this->risk, $this->recharges];
    }
}

/**
 * Solución (ruta completa con objetivos)
 */
class Solution
{
    /** @var int[] */
    public array $route;
    public Objectives $objectives;
    public bool $feasible;
    public int $rank;
    public float $crowdingDistance;

    public function __construct(array $route, Objectives $objectives, bool $feasible = true)
    {
        $this->route = $route;
        $this->objectives = $objectives;
        $this->feasible = $feasible;
        $this->rank = 0;
        $this->crowdingDistance = 0;
    }

    public function dominates(Solution $other): bool
    {
        if (!$this->feasible) return false;
        if (!$other->feasible) return true;
        return $this->objectives->dominates($other->objectives);
    }

    public function toArray(): array
    {
        return [
            'route' => $this->route,
            'objectives' => $this->objectives->toArray(),
            'feasible' => $this->feasible
        ];
    }

    public function __clone()
    {
        $this->objectives = clone $this->objectives;
    }
}

/**
 * Frente de Pareto (conjunto de soluciones no dominadas)
 */
class ParetoFront
{
    /** @var Solution[] */
    private array $solutions = [];

    /**
     * Añade una solución si no está dominada
     * @return bool True si se añadió
     */
    public function add(Solution $solution): bool
    {
        if (!$solution->feasible) return false;

        // Verificar si está dominada por alguna existente
        foreach ($this->solutions as $existing) {
            if ($existing->dominates($solution)) {
                return false;
            }
        }

        // Eliminar las que son dominadas por la nueva
        $this->solutions = array_filter($this->solutions, function($existing) use ($solution) {
            return !$solution->dominates($existing);
        });
        $this->solutions = array_values($this->solutions);

        $this->solutions[] = $solution;
        return true;
    }

    /**
     * Combina con otro frente
     */
    public function merge(ParetoFront $other): void
    {
        foreach ($other->getSolutions() as $solution) {
            $this->add(clone $solution);
        }
    }

    /**
     * @return Solution[]
     */
    public function getSolutions(): array
    {
        return $this->solutions;
    }

    public function size(): int
    {
        return count($this->solutions);
    }

    /**
     * Calcula el hipervolumen (aproximación 2D con distancia y riesgo)
     */
    public function hypervolume(array $refPoint = [1000, 100]): float
    {
        if (empty($this->solutions)) return 0;

        // Proyección 2D: distancia y riesgo
        $points = array_map(function($s) {
            return [$s->objectives->distance, $s->objectives->risk];
        }, $this->solutions);

        // Ordenar por primera coordenada
        usort($points, fn($a, $b) => $a[0] <=> $b[0]);

        $hv = 0;
        $prevY = $refPoint[1];

        foreach ($points as $p) {
            if ($p[0] < $refPoint[0] && $p[1] < $prevY) {
                $hv += ($refPoint[0] - $p[0]) * ($prevY - $p[1]);
                $prevY = $p[1];
            }
        }

        return $hv;
    }

    /**
     * Calcula la diversidad (distancia media entre soluciones consecutivas)
     */
    public function diversity(): float
    {
        if (count($this->solutions) < 2) return 0;

        // Ordenar por distancia
        $sorted = $this->solutions;
        usort($sorted, fn($a, $b) => $a->objectives->distance <=> $b->objectives->distance);

        $totalDist = 0;
        for ($i = 1; $i < count($sorted); $i++) {
            $v1 = $sorted[$i - 1]->objectives->toVector();
            $v2 = $sorted[$i]->objectives->toVector();
            $totalDist += sqrt(
                pow($v1[0] - $v2[0], 2) + 
                pow($v1[1] - $v2[1], 2) + 
                pow($v1[2] - $v2[2], 2)
            );
        }

        return $totalDist / (count($sorted) - 1);
    }

    public function toArray(): array
    {
        return array_map(fn($s) => $s->toArray(), $this->solutions);
    }
}

/**
 * Grafo completo con zonas no-fly
 */
class Graph
{
    /** @var Node[] */
    public array $nodes = [];
    
    /** @var Edge[][] Matriz de adyacencia */
    public array $edges = [];
    
    /** @var Polygon[] */
    public array $noFlyZones = [];
    
    public float $mapSize;

    public function __construct(float $mapSize = 100)
    {
        $this->mapSize = $mapSize;
    }

    public function addNode(Node $node): void
    {
        $this->nodes[$node->id] = $node;
    }

    public function addNoFlyZone(Polygon $polygon): void
    {
        $this->noFlyZones[] = $polygon;
    }

    /**
     * Construye todas las aristas del grafo completo
     */
    public function buildEdges(): void
    {
        $n = count($this->nodes);
        $this->edges = [];

        foreach ($this->nodes as $i => $nodeI) {
            $this->edges[$i] = [];
            foreach ($this->nodes as $j => $nodeJ) {
                if ($i === $j) continue;

                $segment = new Segment($nodeI->position, $nodeJ->position);
                $distance = $segment->length();
                $risk = $this->calculateRisk($segment);
                $battery = $distance / 100; // Consumo proporcional
                $valid = $this->isEdgeValid($segment);

                $this->edges[$i][$j] = new Edge(
                    $i, $j,
                    new EdgeWeight($distance, $risk, $battery),
                    $valid
                );
            }
        }
    }

    /**
     * Verifica si una arista no cruza zonas no-fly
     */
    private function isEdgeValid(Segment $segment): bool
    {
        foreach ($this->noFlyZones as $zone) {
            if (Geometry::segmentIntersectsPolygon($segment, $zone)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Calcula el riesgo de una arista basado en proximidad a zonas no-fly
     */
    private function calculateRisk(Segment $segment): float
    {
        $minDist = PHP_FLOAT_MAX;
        
        foreach ($this->noFlyZones as $zone) {
            $dist = Geometry::segmentToPolygonDistance($segment, $zone);
            $minDist = min($minDist, $dist);
        }

        // Riesgo normalizado: 1.0 si dist < 1, 0.0 si dist > 20
        if ($minDist < 1) return 1.0;
        if ($minDist > 20) return 0.0;
        return 1.0 - ($minDist - 1) / 19;
    }

    public function getEdge(int $from, int $to): ?Edge
    {
        return $this->edges[$from][$to] ?? null;
    }

    public function getNode(int $id): ?Node
    {
        return $this->nodes[$id] ?? null;
    }

    public function nodeCount(): int
    {
        return count($this->nodes);
    }

    /**
     * Obtiene estaciones de recarga
     * @return int[]
     */
    public function getChargingStations(): array
    {
        $stations = [];
        foreach ($this->nodes as $node) {
            if ($node->isCharging) {
                $stations[] = $node->id;
            }
        }
        return $stations;
    }

    /**
     * Evalúa una ruta completa
     */
    public function evaluateRoute(array $route): Solution
    {
        $distance = 0;
        $risk = 0;
        $recharges = 0;
        $battery = 100;
        $feasible = true;

        for ($i = 0; $i < count($route) - 1; $i++) {
            $from = $route[$i];
            $to = $route[$i + 1];
            $edge = $this->getEdge($from, $to);

            if (!$edge || !$edge->valid) {
                $feasible = false;
            }

            if ($edge) {
                $distance += $edge->weight->distance;
                $risk += $edge->weight->risk;
                $battery -= $edge->weight->battery;

                // Verificar recarga
                if ($battery < 20) {
                    $recharges++;
                    $battery = 100;
                }

                // Recarga en estación
                $toNode = $this->getNode($to);
                if ($toNode && $toNode->isCharging && $battery < 80) {
                    $battery = 100;
                }
            }
        }

        return new Solution(
            $route,
            new Objectives($distance, $risk, $recharges),
            $feasible
        );
    }

    public function toArray(): array
    {
        return [
            'nodes' => array_map(fn($n) => $n->toArray(), array_values($this->nodes)),
            'no_fly_zones' => array_map(fn($z) => ['vertices' => $z->toArray()], $this->noFlyZones),
            'map_size' => $this->mapSize
        ];
    }

    public static function fromArray(array $data): Graph
    {
        $graph = new Graph($data['map_size'] ?? 100);

        foreach ($data['nodes'] as $nodeData) {
            $graph->addNode(Node::fromArray($nodeData));
        }

        foreach ($data['no_fly_zones'] as $zoneData) {
            $graph->addNoFlyZone(Polygon::fromArray($zoneData['vertices']));
        }

        $graph->buildEdges();
        return $graph;
    }
}
