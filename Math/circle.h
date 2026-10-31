//
// Created by thong on 25/06/2026.
//

#ifndef UNTITLED_CIRCLE_H
#define UNTITLED_CIRCLE_H
#pragma once
#include "vec2.h"
#include <math.h>
#include "AABB.h"
#ifdef __cplusplus
extern "C" {
#endif
#define PI 3.14159265358979323846
typedef struct {
    double radius;
}Circle;
static inline Circle circleCreate(const double radius) {
    return (Circle){radius};
}
static inline double circleArea(const Circle* c) {
    return (PI * c->radius*c->radius);
}
static inline double circleCircumference(const double radius) {
    return (2 * PI * radius);
}
static inline double circleDiameter(const Circle* circle) {
    return (2 * circle->radius);
}
static inline bool circleContainsPoint(const Circle* c, vec2 center, vec2 point) {
    const double distance = magnitude({point.x - center.x, point.y - center.y});
    return distance <= c->radius;
}
static inline vec2 circleClosestPoint(const Circle* c, const vec2 center, const vec2 point) {
    const vec2 direction = normalize(sub(point, center));
    return add(center, scalarMul(direction, c->radius));
}
static inline AABB circleAABB(const Circle* circle, vec2 center) {
    const vec2 offset = { circle->radius, circle->radius };
    return (AABB){sub(center, offset),add(center, offset)};
}
static inline double circleMass(const Circle* c, const double density) {
    return circleArea(c)*density;
}
static inline double CircleInertia(const Circle* c, const double mass) {
    return pow(c->radius,2)*mass/2;
}
#ifdef __cplusplus
}
#endif
#endif // UNTITLED_CIRCLE_H