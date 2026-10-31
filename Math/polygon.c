//
// Created by thong on 02/07/2026.
//
#include "polygon.h"
#include <stdlib.h>
#include <string.h>
#define EPSILON  1e-12
Polygon polygonCreate(const vec2* vertices, const size_t count) {
    if (count < 3) {
        return polygonEmpty();
    }
    Polygon p;
    p.count = count;
    p.vertices = malloc(count * sizeof(vec2));
    memcpy(p.vertices, vertices, count * sizeof(vec2));
    if (fabs(polygonSignedArea(&p)) == 0){
        return polygonEmpty();
    }
    if (!polygonIsSimple(&p))
    {
        return polygonEmpty();
    }
    if (!polygonIsCCW(&p)) {
        polygonReverseWinding(&p);
    }
    p.convex = polygonIsConvex(&p);
    return p;
}
Polygon polygonCopy(const Polygon* polygon){
    Polygon copy;
    copy.count = polygon->count;
    copy.vertices = malloc(copy.count * sizeof(vec2));
    copy.convex = polygon->convex;
    if (copy.vertices == NULL) {
        copy.count = 0;
        return copy;
    }
    memcpy(copy.vertices, polygon->vertices,copy.count * sizeof(vec2));
    return copy;
}
void polygonDestroy(Polygon* polygon) {
    free(polygon->vertices);
    polygon->vertices = NULL;
    polygon->count = 0;
}
double polygonSignedArea(const Polygon* polygon){
    double area = 0.0;
    for (size_t i = 0; i < polygon->count; i++){
        area += crossProd(polygon->vertices[i],polygon->vertices[(i + 1) % polygon->count]);
    }
    return area * 0.5;
}
double polygonArea(const Polygon* polygon) {
    return fabs(polygonSignedArea(polygon));
}
vec2 polygonCentroid(const Polygon* polygon) {
    const double a = polygonSignedArea(polygon);
    vec2 res = (vec2){0,0};
    for (size_t i = 0; i < polygon->count; i++) {
        const size_t j = (i+1)%polygon->count;
        const double cross = crossProd(polygon->vertices[i],polygon->vertices[(i+1)%polygon->count]);
        res.x += (polygon->vertices[i].x+ polygon->vertices[j].x)*cross;
        res.y += (polygon->vertices[i].y+ polygon->vertices[j].y)*cross;
    }
    res = scalarDiv(res, 6*a);
    return res;
}
AABB polygonComputeLocalAABB(const Polygon* polygon) {
    vec2 min = polygon->vertices[0];
    vec2 max = polygon->vertices[0];
    for (size_t i = 1; i < polygon->count; i++) {
        min = vec2Min(min, polygon->vertices[i]);
        max = vec2Max(max, polygon->vertices[i]);
    }
    return (AABB){min,max};
}
bool polygonContainsPoint(const Polygon* polygon, const vec2 point){
    bool inside = false;
    for (size_t i = 0, j = polygon->count - 1; i < polygon->count; j = i++){
        const vec2 vi = polygon->vertices[i];
        const vec2 vj = polygon->vertices[j];
        if ((vi.y > point.y) != (vj.y > point.y)){
            double xIntersect = vi.x + (point.y - vi.y) * (vj.x - vi.x) / (vj.y - vi.y);
            if (xIntersect > point.x) {
                inside = !inside;
            }
        }
    }
    return inside;
}
vec2 polygonClosestPoint(const Polygon* polygon, const vec2 point){
    if (polygon == NULL || polygon->count == 0) {
        return point;
    }
    vec2 closestVertex = polygon->vertices[0];
    double minLen = magnitude(sub(point, closestVertex));
    for (size_t i = 1; i < polygon->count; i++) {
        vec2 diff = sub(point, polygon->vertices[i]);
        double distSq = magnitude(diff);
        if (distSq < minLen) {
            minLen = distSq;
            closestVertex = polygon->vertices[i];
        }
    }
    return closestVertex;
}
bool polygonIsConvex(const Polygon* polygon){
    if (polygon->count < 3) {
        return false;
    }
    int sign = 0;
    for (size_t i = 0; i < polygon->count; ++i){
        size_t j = (i + 1) % polygon->count;
        size_t k = (i + 2) % polygon->count;
        const vec2 edge1 = sub(polygon->vertices[j],polygon->vertices[i]);
        const vec2 edge2 = sub(polygon->vertices[k],polygon->vertices[j]);
        double cross = crossProd(edge1, edge2);
        if (cross == 0.0) {
            continue;
        }
        if (sign == 0) {
            sign = (cross > 0.0) ? 1 : -1;
        }
        else if ((cross > 0.0 && sign < 0) || (cross < 0.0 && sign > 0)) {
            return false;
        }
    }
    return true;
}
bool polygonIsCCW(const Polygon* p) {
    return polygonSignedArea(p) > 0.0;
}
void polygonReverseWinding(Polygon* polygon){
    for (size_t i = 0; i < polygon->count / 2; ++i){
        size_t j = polygon->count - 1 - i;
        vec2 temp = polygon->vertices[i];
        polygon->vertices[i] = polygon->vertices[j];
        polygon->vertices[j] = temp;
    }
}
bool polygonIsSimple(const Polygon* polygon)
{
    const size_t n = polygon->count;

    for (size_t i = 0; i < n; ++i)
    {
        const size_t iNext = (i + 1) % n;

        for (size_t j = i + 1; j < n; ++j)
        {
            const size_t jNext = (j + 1) % n;

            // Same edge
            if (i == j)
                continue;

            // Adjacent edges share a vertex
            if (iNext == j || jNext == i)
                continue;

            // First and last edges share a vertex
            if (i == 0 && jNext == 0)
                continue;

            if (segmentsIntersect(
                    polygon->vertices[i],
                    polygon->vertices[iNext],
                    polygon->vertices[j],
                    polygon->vertices[jNext]))
            {
                return false;
            }
        }
    }

    return true;
}
bool segmentsIntersect(const vec2 A, const vec2 B, const vec2 C, const vec2 D) {
    const int o1 = orientation(A, B, C);
    const int o2 = orientation(A, B, D);
    const int o3 = orientation(C, D, A);
    const int o4 = orientation(C, D, B);
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    // Collinear cases
    if (o1 == 0 && onSegment(A, B, C)){
        return true;
    }
    if (o2 == 0 && onSegment(A, B, D)){
        return true;
    }
    if (o3 == 0 && onSegment(C, D, A)){
        return true;
    }
    if (o4 == 0 && onSegment(C, D, B)){
        return true;
    }
    return false;
}
static int orientation(const vec2 a, const vec2 b, const vec2 point) {
    const double cross = crossProd(sub(b, a), sub(point, a));
    if (fabs(cross) < EPSILON){
        return 0;
    }
    return (cross > 0.0) ? 1 : -1;
}
static bool onSegment(const vec2 a,  const vec2 b, const vec2 point){
    return point.x >= fmin(a.x, b.x) && point.x <= fmax(a.x, b.x) && point.y >= fmin(a.y, b.y) && point.y <= fmax(a.y, b.y);
}
double polygonMass(const Polygon* polygon, const double density) {
    return polygonArea(polygon)*density;
}
double polygonInertia(const Polygon* polygon, double mass) {
    return mass/polygonArea(polygon);
}
//Later (wait for SAT)
//vec2 polygonSupport(const Polygon* polygon, vec2 direction) {}