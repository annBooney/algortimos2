<?php

namespace DroneRouting\Common;

/**
 * Punto en el plano 2D
 */
class Point
{
    public float $x;
    public float $y;

    public function __construct(float $x, float $y)
    {
        $this->x = $x;
        $this->y = $y;
    }

    public function distanceTo(Point $other): float
    {
        return sqrt(pow($this->x - $other->x, 2) + pow($this->y - $other->y, 2));
    }

    public function toArray(): array
    {
        return [$this->x, $this->y];
    }

    public static function fromArray(array $arr): Point
    {
        return new Point($arr[0], $arr[1]);
    }
}

/**
 * Segmento de línea entre dos puntos
 */
class Segment
{
    public Point $p1;
    public Point $p2;

    public function __construct(Point $p1, Point $p2)
    {
        $this->p1 = $p1;
        $this->p2 = $p2;
    }

    public function length(): float
    {
        return $this->p1->distanceTo($this->p2);
    }
}

/**
 * Polígono (zona no-fly)
 */
class Polygon
{
    /** @var Point[] */
    public array $vertices;

    public function __construct(array $vertices)
    {
        $this->vertices = $vertices;
    }

    /**
     * @return Segment[]
     */
    public function edges(): array
    {
        $edges = [];
        $n = count($this->vertices);
        for ($i = 0; $i < $n; $i++) {
            $edges[] = new Segment($this->vertices[$i], $this->vertices[($i + 1) % $n]);
        }
        return $edges;
    }

    public function toArray(): array
    {
        return array_map(fn(Point $p) => $p->toArray(), $this->vertices);
    }

    public static function fromArray(array $arr): Polygon
    {
        return new Polygon(array_map(fn($v) => Point::fromArray($v), $arr));
    }
}

/**
 * Funciones de geometría computacional
 */
class Geometry
{
    private const EPSILON = 1e-10;

    /**
     * Orientación de tres puntos: -1 (horario), 0 (colineal), 1 (antihorario)
     */
    public static function orientation(Point $p, Point $q, Point $r): int
    {
        $val = ($q->y - $p->y) * ($r->x - $q->x) - ($q->x - $p->x) * ($r->y - $q->y);
        if (abs($val) < self::EPSILON) return 0;
        return $val > 0 ? 1 : -1;
    }

    /**
     * Verifica si el punto q está en el segmento pr (asumiendo colinealidad)
     */
    public static function onSegment(Point $p, Point $q, Point $r): bool
    {
        return $q->x <= max($p->x, $r->x) + self::EPSILON &&
               $q->x >= min($p->x, $r->x) - self::EPSILON &&
               $q->y <= max($p->y, $r->y) + self::EPSILON &&
               $q->y >= min($p->y, $r->y) - self::EPSILON;
    }

    /**
     * Verifica si dos segmentos se intersectan
     */
    public static function segmentsIntersect(Segment $s1, Segment $s2): bool
    {
        $p1 = $s1->p1; $q1 = $s1->p2;
        $p2 = $s2->p1; $q2 = $s2->p2;

        $o1 = self::orientation($p1, $q1, $p2);
        $o2 = self::orientation($p1, $q1, $q2);
        $o3 = self::orientation($p2, $q2, $p1);
        $o4 = self::orientation($p2, $q2, $q1);

        // Caso general
        if ($o1 !== $o2 && $o3 !== $o4) return true;

        // Casos especiales (colineales)
        if ($o1 === 0 && self::onSegment($p1, $p2, $q1)) return true;
        if ($o2 === 0 && self::onSegment($p1, $q2, $q1)) return true;
        if ($o3 === 0 && self::onSegment($p2, $p1, $q2)) return true;
        if ($o4 === 0 && self::onSegment($p2, $q1, $q2)) return true;

        return false;
    }

    /**
     * Verifica si un punto está dentro de un polígono (ray casting)
     */
    public static function pointInPolygon(Point $p, Polygon $polygon): bool
    {
        $n = count($polygon->vertices);
        if ($n < 3) return false;

        // Rayo hacia la derecha
        $extreme = new Point(PHP_INT_MAX, $p->y);
        $ray = new Segment($p, $extreme);

        $count = 0;
        for ($i = 0; $i < $n; $i++) {
            $edge = new Segment($polygon->vertices[$i], $polygon->vertices[($i + 1) % $n]);
            
            if (self::segmentsIntersect($ray, $edge)) {
                // Si el punto es colineal con el edge, verificar si está en el segmento
                if (self::orientation($edge->p1, $p, $edge->p2) === 0) {
                    return self::onSegment($edge->p1, $p, $edge->p2);
                }
                $count++;
            }
        }

        return $count % 2 === 1;
    }

    /**
     * Verifica si un segmento intersecta un polígono (cruza o toca)
     */
    public static function segmentIntersectsPolygon(Segment $segment, Polygon $polygon): bool
    {
        // Verificar intersección con cada arista
        foreach ($polygon->edges() as $edge) {
            if (self::segmentsIntersect($segment, $edge)) {
                return true;
            }
        }

        // Verificar si algún extremo está dentro del polígono
        if (self::pointInPolygon($segment->p1, $polygon) || 
            self::pointInPolygon($segment->p2, $polygon)) {
            return true;
        }

        return false;
    }

    /**
     * Distancia mínima de un punto a un segmento
     */
    public static function pointToSegmentDistance(Point $p, Segment $s): float
    {
        $A = $s->p1;
        $B = $s->p2;
        
        $ABx = $B->x - $A->x;
        $ABy = $B->y - $A->y;
        $APx = $p->x - $A->x;
        $APy = $p->y - $A->y;
        
        $AB_AB = $ABx * $ABx + $ABy * $ABy;
        
        if ($AB_AB < self::EPSILON) {
            return $p->distanceTo($A);
        }
        
        $t = max(0, min(1, ($APx * $ABx + $APy * $ABy) / $AB_AB));
        
        $closest = new Point($A->x + $t * $ABx, $A->y + $t * $ABy);
        return $p->distanceTo($closest);
    }

    /**
     * Distancia mínima de un punto a un polígono
     */
    public static function pointToPolygonDistance(Point $p, Polygon $polygon): float
    {
        if (self::pointInPolygon($p, $polygon)) {
            return 0;
        }

        $minDist = PHP_FLOAT_MAX;
        foreach ($polygon->edges() as $edge) {
            $dist = self::pointToSegmentDistance($p, $edge);
            $minDist = min($minDist, $dist);
        }
        return $minDist;
    }

    /**
     * Distancia mínima de un segmento a un polígono
     */
    public static function segmentToPolygonDistance(Segment $segment, Polygon $polygon): float
    {
        if (self::segmentIntersectsPolygon($segment, $polygon)) {
            return 0;
        }

        $minDist = PHP_FLOAT_MAX;
        
        // Distancia de los extremos del segmento al polígono
        $minDist = min($minDist, self::pointToPolygonDistance($segment->p1, $polygon));
        $minDist = min($minDist, self::pointToPolygonDistance($segment->p2, $polygon));
        
        // Distancia de los vértices del polígono al segmento
        foreach ($polygon->vertices as $vertex) {
            $minDist = min($minDist, self::pointToSegmentDistance($vertex, $segment));
        }
        
        return $minDist;
    }

    /**
     * Envolvente convexa (Andrew's monotone chain algorithm)
     * @param Point[] $points
     * @return Point[]
     */
    public static function convexHull(array $points): array
    {
        $n = count($points);
        if ($n < 3) return $points;

        // Ordenar por x, luego por y
        usort($points, function(Point $a, Point $b) {
            if (abs($a->x - $b->x) < self::EPSILON) {
                return $a->y <=> $b->y;
            }
            return $a->x <=> $b->x;
        });

        $hull = [];

        // Construir mitad inferior
        foreach ($points as $p) {
            while (count($hull) >= 2 && 
                   self::cross($hull[count($hull) - 2], $hull[count($hull) - 1], $p) <= 0) {
                array_pop($hull);
            }
            $hull[] = $p;
        }

        // Construir mitad superior
        $lowerSize = count($hull);
        for ($i = $n - 2; $i >= 0; $i--) {
            while (count($hull) > $lowerSize && 
                   self::cross($hull[count($hull) - 2], $hull[count($hull) - 1], $points[$i]) <= 0) {
                array_pop($hull);
            }
            $hull[] = $points[$i];
        }

        array_pop($hull); // Remover último punto (duplicado del primero)
        return $hull;
    }

    /**
     * Producto cruz 2D: (B-A) × (C-A)
     */
    private static function cross(Point $a, Point $b, Point $c): float
    {
        return ($b->x - $a->x) * ($c->y - $a->y) - ($b->y - $a->y) * ($c->x - $a->x);
    }

    /**
     * Ángulo polar de un punto respecto a un centro
     */
    public static function polarAngle(Point $center, Point $point): float
    {
        return atan2($point->y - $center->y, $point->x - $center->x);
    }
}
