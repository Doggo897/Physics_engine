//
// Created by thong on 02/07/2026.
//

#ifndef UNTITLED_POLYGON_H
#define UNTITLED_POLYGON_H
#pragma once
#include <stddef.h>
#include <stdbool.h>
#include "vec2.h"
#include "AABB.h"
#include "transform.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    vec2* vertices;
    size_t count;
    bool convex;
}Polygon;
Polygon polygonCreate(const vec2* vertices, size_t count);
Polygon polygonCopy(const Polygon* polygon);
void polygonDestroy(Polygon* polygon);
double polygonSignedArea(const Polygon* polygon);
double polygonArea(const Polygon* polygon);
vec2 polygonCentroid(const Polygon* polygon);
bool polygonContainsPoint(const Polygon* polygon, vec2 point);
vec2 polygonClosestPoint(const Polygon* polygon, vec2 point);
bool polygonIsConvex(const Polygon* polygon);
bool polygonIsCCW(const Polygon* p);
void polygonReverseWinding(Polygon* polygon);
bool polygonIsSimple(const Polygon* polygon);
static int orientation(vec2 a, vec2 b, vec2 point);
static bool onSegment(vec2 a, vec2 b, vec2 point);
bool segmentsIntersect(vec2 A, vec2 B, vec2 C,  vec2 D);
double polygonMass(const Polygon* polygon, double density);
double polygonInertia(const Polygon* polygon, double mass);
vec2 polygonSupport(const Polygon* polygon, vec2 direction);
static inline size_t polygonVertexCount(const Polygon* polygon) {
    return polygon->count;
}
static inline vec2 getVertex(const Polygon* polygon, size_t index) {
    return polygon->vertices[index];
}
static inline vec2 polygonEdgeNormal(const Polygon* polygon, const size_t edgeIndex) {
    const vec2 a = polygon->vertices[edgeIndex];
    const vec2 b = polygon->vertices[(edgeIndex + 1) % polygon->count];
    const vec2 edge = sub(b, a);
    const vec2 normal = { -edge.y, edge.x };
    return normalize(normal);
}
static inline Polygon polygonEmpty(void){
    return (Polygon){NULL, 0, false};
}
AABB polygonComputeLocalAABB(const Polygon* polygon);
#ifdef __cplusplus
}
#endif
#endif //UNTITLED_POLYGON_H