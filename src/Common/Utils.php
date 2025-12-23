<?php

namespace DroneRouting\Common;

/**
 * Utilidades para generación de instancias y experimentación
 */
class Utils
{
    /**
     * Genera una instancia aleatoria del problema
     */
    public static function generateInstance(
        int $numNodes = 15,
        int $numZones = 3,
        float $mapSize = 100,
        ?int $seed = null
    ): Graph {
        if ($seed !== null) {
            mt_srand($seed);
        }

        $graph = new Graph($mapSize);
        
        // Generar zonas no-fly primero
        $zones = [];
        for ($i = 0; $i < $numZones; $i++) {
            $zone = self::generateConvexPolygon($mapSize);
            $zones[] = $zone;
            $graph->addNoFlyZone($zone);
        }

        // Hub central (nodo 0)
        $hubPos = self::findValidPosition($mapSize / 2, $mapSize / 2, $mapSize, $zones, 5);
        $graph->addNode(new Node(0, $hubPos, true, false));

        // Generar nodos (10-20% son estaciones de recarga)
        $numCharging = max(1, (int)($numNodes * 0.15));
        $chargingIndices = self::randomSample(range(1, $numNodes - 1), $numCharging);

        for ($i = 1; $i < $numNodes; $i++) {
            $pos = self::findValidPosition(
                mt_rand(5, (int)$mapSize - 5),
                mt_rand(5, (int)$mapSize - 5),
                $mapSize,
                $zones,
                3
            );
            $isCharging = in_array($i, $chargingIndices);
            $graph->addNode(new Node($i, $pos, false, $isCharging));
        }

        $graph->buildEdges();
        return $graph;
    }

    /**
     * Genera un polígono convexo aleatorio
     */
    private static function generateConvexPolygon(float $mapSize): Polygon
    {
        $centerX = mt_rand((int)($mapSize * 0.2), (int)($mapSize * 0.8));
        $centerY = mt_rand((int)($mapSize * 0.2), (int)($mapSize * 0.8));
        $radius = mt_rand((int)($mapSize * 0.05), (int)($mapSize * 0.15));

        // Generar puntos alrededor del centro
        $numVertices = mt_rand(4, 7);
        $angles = [];
        for ($i = 0; $i < $numVertices; $i++) {
            $angles[] = mt_rand(0, 360) * M_PI / 180;
        }
        sort($angles);

        $vertices = [];
        foreach ($angles as $angle) {
            $r = $radius * (0.7 + mt_rand(0, 30) / 100);
            $vertices[] = new Point(
                $centerX + $r * cos($angle),
                $centerY + $r * sin($angle)
            );
        }

        return new Polygon($vertices);
    }

    /**
     * Encuentra una posición válida (no dentro de zonas no-fly)
     */
    private static function findValidPosition(
        float $x, 
        float $y, 
        float $mapSize, 
        array $zones,
        float $minDist
    ): Point {
        $attempts = 0;
        $maxAttempts = 100;

        while ($attempts < $maxAttempts) {
            $point = new Point($x, $y);
            $valid = true;

            foreach ($zones as $zone) {
                if (Geometry::pointInPolygon($point, $zone) || 
                    Geometry::pointToPolygonDistance($point, $zone) < $minDist) {
                    $valid = false;
                    break;
                }
            }

            if ($valid) return $point;

            // Generar nueva posición aleatoria
            $x = mt_rand(5, (int)$mapSize - 5);
            $y = mt_rand(5, (int)$mapSize - 5);
            $attempts++;
        }

        // Si no encuentra, devolver la última posición
        return new Point($x, $y);
    }

    /**
     * Selecciona k elementos aleatorios de un array
     */
    private static function randomSample(array $arr, int $k): array
    {
        shuffle($arr);
        return array_slice($arr, 0, $k);
    }

    /**
     * Guarda una instancia en JSON
     */
    public static function saveInstance(Graph $graph, string $filename): void
    {
        $json = json_encode($graph->toArray(), JSON_PRETTY_PRINT);
        file_put_contents($filename, $json);
    }

    /**
     * Carga una instancia desde JSON
     */
    public static function loadInstance(string $filename): Graph
    {
        $json = file_get_contents($filename);
        $data = json_decode($json, true);
        return Graph::fromArray($data);
    }

    /**
     * Guarda un frente de Pareto en JSON
     */
    public static function saveParetoFront(ParetoFront $front, string $filename): void
    {
        $json = json_encode($front->toArray(), JSON_PRETTY_PRINT);
        file_put_contents($filename, $json);
    }

    /**
     * Mide el tiempo de ejecución de una función
     * @return array [result, time_seconds]
     */
    public static function measureTime(callable $fn): array
    {
        $start = microtime(true);
        $result = $fn();
        $end = microtime(true);
        return [$result, $end - $start];
    }

    /**
     * Mide la memoria usada por una función
     * @return array [result, memory_mb]
     */
    public static function measureMemory(callable $fn): array
    {
        $startMem = memory_get_usage(true);
        $result = $fn();
        $peakMem = memory_get_peak_usage(true);
        return [$result, ($peakMem - $startMem) / 1024 / 1024];
    }

    /**
     * Ejecuta una función midiendo tiempo y memoria
     * @return array [result, time, memory]
     */
    public static function benchmark(callable $fn): array
    {
        gc_collect_cycles();
        $startMem = memory_get_usage(true);
        $start = microtime(true);
        
        $result = $fn();
        
        $time = microtime(true) - $start;
        $peakMem = memory_get_peak_usage(true);
        $memory = ($peakMem - $startMem) / 1024 / 1024;

        return [$result, $time, max(0, $memory)];
    }

    /**
     * Calcula media y desviación estándar
     */
    public static function stats(array $values): array
    {
        if (empty($values)) {
            return ['mean' => 0, 'std' => 0];
        }

        $n = count($values);
        $mean = array_sum($values) / $n;
        
        $variance = 0;
        foreach ($values as $v) {
            $variance += pow($v - $mean, 2);
        }
        $std = $n > 1 ? sqrt($variance / ($n - 1)) : 0;

        return ['mean' => $mean, 'std' => $std];
    }
}
