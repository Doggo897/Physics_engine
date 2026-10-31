//
// Created by thong on 06/01/2026.
//
#ifndef C_PROJECTS_MAT2_H
#define C_PROJECTS_MAT2_H
#pragma once
#include <math.h>
#include "vec2.h"
#ifdef __cplusplus
extern "C" {
#endif
#define PI 3.14159265358979323846
// form:  [m00,m01]
//        [m10,m11]
typedef struct {double m00,m01,m10,m11;} mat2;
//return identity matrix
static inline mat2 identity() {
    return (mat2){1,0,0,1};
}
//returning a scaling matrix
static inline mat2 scaleMat2(const double sx, const double sy) {
    return (mat2){sx,0,0,sy};
}
//turning a matrix into an identity matrix
static inline void mat2SetIdentity(mat2 m){
    m.m00=1.0;
    m.m01=0.0;
    m.m10=0.0;
    m.m11=1.0;
}
//Mutating all of a matrix's elements to zero
static inline void mat2SetZero(mat2 m) {
    m.m00=0.0;
    m.m01=0.0;
    m.m10=0.0;
    m.m11=0.0;
}
static inline void mat2SetRotation(mat2* m, const double angle) {
    const double c = cos(angle);
    const double s = sin(angle);
    m->m00 = c;
    m->m01 = -s;
    m->m10 = s;
    m->m11 = c;
}
static inline vec2 mat2MulVec2 (const mat2 m, const vec2 v) {
    return (vec2){m.m00*v.x+m.m01*v.y,m.m10*v.x+m.m11*v.y};
}
static inline mat2 transpose(const mat2 m) {
    return (mat2){m.m00,m.m10,m.m01,m.m11};
}
static inline mat2 multiply(const mat2 a, const mat2 b) {
    return (mat2){a.m00*b.m00+a.m01*b.m10,a.m00*b.m01+a.m01*b.m11, a.m10*b.m00+a.m11*b.m10, a.m10*b.m01+a.m11*b.m11};
}
#ifdef __cplusplus
}
#endif
#endif //C_PROJECTS_MAT2_H