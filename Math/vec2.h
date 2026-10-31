//
// Created by thong on 17/12/2025.
//
#ifndef C_PROJECTS_VEC2_H
#define C_PROJECTS_VEC2_H
#pragma once
#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif
// form: (x)
//       (y)
typedef struct {double x,y;} vec2;
extern const vec2 zeroVec = {0,0};
static inline vec2 add(const vec2 a, const vec2 b) {
    vec2 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    return res;
}
static inline vec2 sub(const vec2 a, const vec2 b) {
    vec2 res;
    res.x = a.x - b.x;
    res.y = a.y - b.y;
    return res;
}
static inline vec2 scalarMul(const vec2 a, const double b) {
    vec2 res;
    res.x = a.x * b;
    res.y = a.y * b;
    return res;
}
static inline vec2 scalarDiv(const vec2 a, const double b) {
    vec2 res;
    res.x = a.x / b;
    res.y = a.y / b;
    return res;
}
//dot product
static inline double dotProd(const vec2 a, const vec2 b) {
    return (a.x * b.x) + (a.y * b.y);
}
//cross product
static inline double crossProd(const vec2 a, const vec2 b) {
    return a.x*b.y - a.y*b.x;
}
//find magnitude of a vector
static inline double magnitude(const vec2 a) {
    return sqrt(a.x*a.x + a.y*a.y);
}
static inline vec2 normalize(const vec2 a) {
    const double mag = magnitude(a);
    if (mag == 0.0) return(vec2){0,0};   // avoid division by zero (only case is origin so return origin)
    return scalarDiv(a, mag);
}
static inline vec2 vec2Max(vec2 a, vec2 b) {
    return (vec2){a.x > b.x ? a.x : b.x,a.y > b.y ? a.y : b.y};
}
static inline vec2 vec2Min(vec2 a, vec2 b){
    return (vec2){a.x < b.x ? a.x : b.x,a.y < b.y ? a.y : b.y};
}
#ifdef __cplusplus
}
#endif
#endif //C_PROJECTS_VEC2_H