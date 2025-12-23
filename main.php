#!/usr/bin/env php
<?php

require_once __DIR__ . '/src/Common/Geometry.php';
require_once __DIR__ . '/src/Common/DataStructures.php';
require_once __DIR__ . '/src/Common/Utils.php';
require_once __DIR__ . '/src/ExactBB/BranchAndBound.php';
require_once __DIR__ . '/src/GeoHeuristic/GeometricHeuristics.php';
require_once __DIR__ . '/src/Metaheuristic/NSGA2.php';

use DroneRouting\Common\{Graph, Utils, ParetoFront};
use DroneRouting\ExactBB\BranchAndBound;
use DroneRouting\GeoHeuristic\GeometricHeuristics;
use DroneRouting\Metaheuristic\NSGA2;

/**
 * CLI para el sistema de optimización de rutas de drones
 */
class DroneRoutingCLI
{
    public function run(array $argv): void
    {
        if (count($argv) < 2) {
            $this->showHelp();
            return;
        }

        $command = $argv[1];

        switch ($command) {
            case 'generate':
                $this->generate($argv);
                break;
            case 'solve':
                $this->solve($argv);
                break;
            case 'experiment':
                $this->experiment($argv);
                break;
            case 'help':
            default:
                $this->showHelp();
                break;
        }
    }

    private function showHelp(): void
    {
        echo <<<HELP
Optimización Multi-objetivo de Rutas de Drones
=============================================

Uso: php main.php <comando> [opciones]

Comandos:
  generate    Genera una instancia del problema
  solve       Resuelve una instancia
  experiment  Ejecuta experimentos comparativos
  help        Muestra esta ayuda

Ejemplos:
  php main.php generate --nodes 15 --zones 3 --seed 42 --output instance.json
  php main.php solve instance.json --algorithm all
  php main.php experiment --replicas 5

Opciones de generate:
  --nodes N       Número de nodos (default: 15)
  --zones Z       Número de zonas no-fly (default: 3)
  --seed S        Semilla aleatoria (default: ninguna)
  --output FILE   Archivo de salida (default: instance.json)

Opciones de solve:
  --algorithm A   Algoritmo: exact, heuristic, meta, all (default: all)
  --output FILE   Archivo de salida (default: solution.json)

Opciones de experiment:
  --replicas R    Número de réplicas (default: 5)
  --max-bb N      Máximo de nodos para Branch & Bound (default: 15)

HELP;
    }

    private function generate(array $argv): void
    {
        $nodes = 15;
        $zones = 3;
        $seed = null;
        $output = 'instances/instance.json';

        for ($i = 2; $i < count($argv); $i++) {
            switch ($argv[$i]) {
                case '--nodes':
                    $nodes = (int)($argv[++$i] ?? 15);
                    break;
                case '--zones':
                    $zones = (int)($argv[++$i] ?? 3);
                    break;
                case '--seed':
                    $seed = (int)($argv[++$i] ?? null);
                    break;
                case '--output':
                    $output = $argv[++$i] ?? 'instance.json';
                    break;
            }
        }

        echo "Generando instancia con $nodes nodos y $zones zonas no-fly...\n";
        
        $graph = Utils::generateInstance($nodes, $zones, 100, $seed);
        
        // Crear directorio si no existe
        $dir = dirname($output);
        if (!is_dir($dir)) {
            mkdir($dir, 0755, true);
        }
        
        Utils::saveInstance($graph, $output);
        
        echo "✓ Instancia guardada en: $output\n";
        echo "  - Nodos: $nodes\n";
        echo "  - Zonas no-fly: $zones\n";
        echo "  - Estaciones de recarga: " . count($graph->getChargingStations()) . "\n";
    }

    private function solve(array $argv): void
    {
        if (count($argv) < 3) {
            echo "Error: Especifica el archivo de instancia\n";
            echo "Uso: php main.php solve <instance.json> [--algorithm all]\n";
            return;
        }

        $instanceFile = $argv[2];
        $algorithm = 'all';
        $output = 'results/solution.json';

        for ($i = 3; $i < count($argv); $i++) {
            switch ($argv[$i]) {
                case '--algorithm':
                    $algorithm = $argv[++$i] ?? 'all';
                    break;
                case '--output':
                    $output = $argv[++$i] ?? 'solution.json';
                    break;
            }
        }

        if (!file_exists($instanceFile)) {
            echo "Error: No se encuentra el archivo: $instanceFile\n";
            return;
        }

        echo "Cargando instancia: $instanceFile\n";
        $graph = Utils::loadInstance($instanceFile);
        echo "  - Nodos: {$graph->nodeCount()}\n";
        echo "  - Zonas no-fly: " . count($graph->noFlyZones) . "\n\n";

        $results = [];

        // Ejecutar algoritmos
        if ($algorithm === 'all' || $algorithm === 'exact') {
            if ($graph->nodeCount() <= 15) {
                echo "Ejecutando Branch & Bound...\n";
                [$front, $time, $mem] = Utils::benchmark(function() use ($graph) {
                    $bb = new BranchAndBound($graph);
                    return $bb->solve();
                });
                $results['exact'] = [
                    'front' => $front,
                    'time' => $time,
                    'memory' => $mem
                ];
                echo "  ✓ Completado en " . number_format($time, 3) . "s\n";
                echo "  - Soluciones en frente: {$front->size()}\n\n";
            } else {
                echo "⚠ Branch & Bound omitido (N > 15)\n\n";
            }
        }

        if ($algorithm === 'all' || $algorithm === 'heuristic') {
            echo "Ejecutando Heurísticas Geométricas...\n";
            [$front, $time, $mem] = Utils::benchmark(function() use ($graph) {
                $heuristics = new GeometricHeuristics($graph);
                return $heuristics->solve();
            });
            $results['heuristic'] = [
                'front' => $front,
                'time' => $time,
                'memory' => $mem
            ];
            echo "  ✓ Completado en " . number_format($time, 3) . "s\n";
            echo "  - Soluciones en frente: {$front->size()}\n\n";
        }

        if ($algorithm === 'all' || $algorithm === 'meta') {
            echo "Ejecutando NSGA-II...\n";
            [$front, $time, $mem] = Utils::benchmark(function() use ($graph) {
                $nsga2 = new NSGA2($graph, 100, 200);
                return $nsga2->solve();
            });
            $results['meta'] = [
                'front' => $front,
                'time' => $time,
                'memory' => $mem
            ];
            echo "  ✓ Completado en " . number_format($time, 3) . "s\n";
            echo "  - Soluciones en frente: {$front->size()}\n\n";
        }

        // Mostrar resumen
        echo "=== RESUMEN ===\n";
        foreach ($results as $name => $data) {
            /** @var ParetoFront $front */
            $front = $data['front'];
            $hv = $front->hypervolume();
            $div = $front->diversity();
            
            echo sprintf("%-12s | Tiempo: %7.3fs | Soluciones: %3d | HV: %8.2f | Div: %6.2f\n",
                ucfirst($name),
                $data['time'],
                $front->size(),
                $hv,
                $div
            );
        }

        // Guardar resultados
        $dir = dirname($output);
        if (!is_dir($dir)) {
            mkdir($dir, 0755, true);
        }

        $outputData = [];
        foreach ($results as $name => $data) {
            $outputData[$name] = [
                'time' => $data['time'],
                'memory' => $data['memory'],
                'pareto_front' => $data['front']->toArray(),
                'hypervolume' => $data['front']->hypervolume(),
                'diversity' => $data['front']->diversity()
            ];
        }

        file_put_contents($output, json_encode($outputData, JSON_PRETTY_PRINT));
        echo "\n✓ Resultados guardados en: $output\n";
    }

    private function experiment(array $argv): void
    {
        $replicas = 5;
        $maxBBNodes = 15;

        for ($i = 2; $i < count($argv); $i++) {
            switch ($argv[$i]) {
                case '--replicas':
                    $replicas = (int)($argv[++$i] ?? 5);
                    break;
                case '--max-bb':
                    $maxBBNodes = (int)($argv[++$i] ?? 15);
                    break;
            }
        }

        echo "=== EXPERIMENTOS DE OPTIMIZACIÓN DE RUTAS DE DRONES ===\n\n";

        // Configuración de instancias
        $instances = [
            ['nodes' => 10, 'zones' => 3, 'seed' => 42],
            ['nodes' => 15, 'zones' => 3, 'seed' => 123],
            ['nodes' => 20, 'zones' => 4, 'seed' => 456],
            ['nodes' => 25, 'zones' => 4, 'seed' => 789],
        ];

        $allResults = [];

        foreach ($instances as $config) {
            $n = $config['nodes'];
            echo "--- Instancia N=$n ---\n";

            // Generar instancia
            $graph = Utils::generateInstance($config['nodes'], $config['zones'], 100, $config['seed']);
            
            // Guardar instancia
            $instanceFile = "instances/instance_n{$n}.json";
            if (!is_dir('instances')) mkdir('instances', 0755, true);
            Utils::saveInstance($graph, $instanceFile);
            echo "  Instancia guardada: $instanceFile\n";

            $instanceResults = [
                'nodes' => $n,
                'zones' => $config['zones'],
                'algorithms' => []
            ];

            // Ejecutar cada algoritmo
            $algorithms = [
                'exact' => fn($g) => (new BranchAndBound($g))->solve(),
                'heuristic' => fn($g) => (new GeometricHeuristics($g))->solve(),
                'meta' => fn($g) => (new NSGA2($g, 100, 200))->solve(),
            ];

            foreach ($algorithms as $name => $solver) {
                // Omitir exact para instancias grandes
                if ($name === 'exact' && $n > $maxBBNodes) {
                    echo "  $name: OMITIDO (N > $maxBBNodes)\n";
                    continue;
                }

                echo "  $name: ";
                $times = [];
                $memories = [];
                $hvs = [];
                $divs = [];
                $frontSizes = [];

                for ($rep = 0; $rep < $replicas; $rep++) {
                    [$front, $time, $mem] = Utils::benchmark(fn() => $solver($graph));
                    $times[] = $time;
                    $memories[] = $mem;
                    $hvs[] = $front->hypervolume();
                    $divs[] = $front->diversity();
                    $frontSizes[] = $front->size();
                    echo ".";
                }
                echo "\n";

                $instanceResults['algorithms'][$name] = [
                    'time' => Utils::stats($times),
                    'memory' => Utils::stats($memories),
                    'hypervolume' => Utils::stats($hvs),
                    'diversity' => Utils::stats($divs),
                    'front_size' => Utils::stats($frontSizes),
                ];

                $timeStats = Utils::stats($times);
                echo sprintf("    Tiempo: %.3f ± %.3f s | HV: %.2f | Soluciones: %.1f\n",
                    $timeStats['mean'],
                    $timeStats['std'],
                    Utils::stats($hvs)['mean'],
                    Utils::stats($frontSizes)['mean']
                );
            }

            $allResults[] = $instanceResults;
            echo "\n";
        }

        // Guardar resultados
        if (!is_dir('results')) mkdir('results', 0755, true);
        $resultsFile = 'results/experiment_results.json';
        file_put_contents($resultsFile, json_encode($allResults, JSON_PRETTY_PRINT));
        echo "✓ Resultados guardados en: $resultsFile\n";

        // Mostrar tabla resumen
        echo "\n=== TABLA RESUMEN ===\n";
        echo str_repeat("-", 80) . "\n";
        echo sprintf("%-6s | %-12s | %-15s | %-12s | %-12s\n", 
            "N", "Algoritmo", "Tiempo (s)", "Hipervolumen", "Soluciones");
        echo str_repeat("-", 80) . "\n";

        foreach ($allResults as $result) {
            foreach ($result['algorithms'] as $algo => $data) {
                echo sprintf("%-6d | %-12s | %6.3f ± %5.3f | %12.2f | %12.1f\n",
                    $result['nodes'],
                    $algo,
                    $data['time']['mean'],
                    $data['time']['std'],
                    $data['hypervolume']['mean'],
                    $data['front_size']['mean']
                );
            }
        }
        echo str_repeat("-", 80) . "\n";
    }
}

// Ejecutar CLI
$cli = new DroneRoutingCLI();
$cli->run($argv);
