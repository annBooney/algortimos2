# Makefile para Optimización Multi-objetivo de Rutas de Drones (PHP)

.PHONY: all generate run solve test clean help

# Configuración por defecto
PHP = php
INSTANCE ?= instances/instance_n15.json
ALGORITHM ?= all

all: generate run

# Genera las instancias de prueba
generate:
	@echo "Generando instancias de prueba..."
	@mkdir -p instances
	$(PHP) main.php generate --nodes 10 --zones 3 --seed 42 --output instances/instance_n10.json
	$(PHP) main.php generate --nodes 15 --zones 3 --seed 123 --output instances/instance_n15.json
	$(PHP) main.php generate --nodes 20 --zones 4 --seed 456 --output instances/instance_n20.json
	$(PHP) main.php generate --nodes 25 --zones 4 --seed 789 --output instances/instance_n25.json
	@echo "✓ Instancias generadas en instances/"

# Ejecuta los experimentos completos
run: generate
	@echo "Ejecutando experimentos..."
	$(PHP) main.php experiment --replicas 5 --max-bb 15
	@echo "✓ Experimentos completados"

# Resuelve una instancia específica
solve:
	$(PHP) main.php solve $(INSTANCE) --algorithm $(ALGORITHM)

# Prueba rápida con instancia pequeña
test:
	@echo "Ejecutando prueba rápida..."
	$(PHP) main.php generate --nodes 8 --zones 2 --seed 999 --output instances/test.json
	$(PHP) main.php solve instances/test.json --algorithm all
	@echo "✓ Prueba completada"

# Limpia archivos generados
clean:
	rm -rf instances/*.json
	rm -rf results/*.json
	@echo "✓ Archivos limpiados"

# Muestra ayuda
help:
	$(PHP) main.php help

# Ejemplos de uso
example-generate:
	$(PHP) main.php generate --nodes 20 --zones 4 --seed 42 --output instances/custom.json

example-solve-exact:
	$(PHP) main.php solve instances/instance_n10.json --algorithm exact

example-solve-heuristic:
	$(PHP) main.php solve instances/instance_n15.json --algorithm heuristic

example-solve-meta:
	$(PHP) main.php solve instances/instance_n20.json --algorithm meta
