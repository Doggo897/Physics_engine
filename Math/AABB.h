//
// Created by thong on 19/06/2026.
//

#ifndef UNTITLED_AABB_H
#define UNTITLED_AABB_H
#pragma once
#include <math.h>
#include "vec2.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    vec2 min;
    vec2 max;
} AABB;
static inline AABB create(const vec2 a, const vec2 b) {
    return (AABB){vec2Min(a,b), vec2Max(a,b)};
}
static inline AABB aabbFromCenter(const vec2 center, const vec2 reach) {
    return (AABB){sub(center,reach), add(center,reach)};
}
static inline double aabbHeight(const AABB* box) {
    return (double)(box->max.y - box->min.y);
}
static inline double aabbWidth(const AABB* box) {
    return (double)(box->max.x - box->min.x);
}
static inline double aabbArea(const AABB* box) {
    return (double){aabbWidth(box)*aabbHeight(box)};
}
static inline vec2 aabbCenter(const AABB* box) {
    return (vec2){(box->min.x+box->max.x)/2,(box->min.y+box->max.y)/2};
}
static inline vec2 aabbExtent(const AABB* box) {
    const vec2 center = aabbCenter(box);
    return (vec2){box->max.x - center.x,box->max.y - center.y};
}
static inline bool containsPoint(const AABB* box, vec2 point) {
    return (point.x >= box->min.x && point.x <= box->max.x && point.y >= box->min.y && point.y <= box->max.y);
}
//checks if box1 contains box2
static inline bool containsAABB(const AABB* box1, const AABB* box2) {
    return (box1->max.x>=box2->max.x && box1->max.y>=box2->max.y && box2->min.x>=box1->min.x && box2->min.y>=box1->min.y);
}
static inline bool overlapsAABB(const AABB* box1, const AABB* box2) {
    return (box1->max.x>=box2->max.x || box1->max.y>=box2->max.y || box2->min.x>=box1->min.x || box2->min.y>=box1->min.y);
}
static inline AABB aabbExpand(const AABB* box, double amount) {
    const vec2 offset = { amount, amount };
    return (AABB){sub(box->min, offset),add(box->max, offset)};
}
static inline AABB aabbUnion(const AABB* a, const AABB* b) {
    AABB res;
    res.min = vec2Min(a->min, b->min);
    res.max = vec2Max(a->max, b->max);
    return res;
}
static inline AABB translateAABB(const AABB* box, vec2 offset) {
    return (AABB){add(box->min, offset),add(box->max, offset)};
}
static inline double perimeter(const AABB* box) {
    return (double){2*(aabbWidth(box)+aabbHeight(box))};
}
static inline double clamp(double x, double min, double max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}
static inline vec2 aabbClosestPoint(const AABB* box, vec2 point) {
    return (vec2){clamp(point.x, box->min.x, box->max.x),clamp(point.y, box->min.y, box->max.y)};
}
//Currently not developed yet, wait till collision, used in collision detection, t stands for distance to hit the box
static inline bool raycast(const AABB* box, vec2 origin, vec2 direction, double* t) {
    return 0;
}
#ifdef __cplusplus
}
#endif
#endif //UNTITLED_AABB_H