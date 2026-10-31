//
// Created by thong on 10/02/2026.
//

#ifndef MYPROJECT_TRANSFORM_H
#define MYPROJECT_TRANSFORM_H
#pragma once
#include "mat2.h"
#include "vec2.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    vec2 position; // translation
    mat2 rotation; // orientation
}Transform;
inline vec2 transformPoint(const Transform* t, const vec2 local) {
    const vec2 rotated = mat2MulVec2(t->rotation, local);
    return add(rotated, t->position);
}
inline Transform transformMake(const vec2 pos, const double angle) {
    mat2 rotation = identity();
    mat2SetRotation(&rotation, angle);
    return (Transform){pos, rotation};
}
inline vec2 inverseTransformPoint(const Transform* t, vec2 world) {
    world = sub(world, t->position);
    return mat2MulVec2(transpose(t->rotation), world);
}
static inline Transform transformMul(const Transform* a, const Transform* b) {
    return (Transform){.position = add(mat2MulVec2(a->rotation, b->position), a->position), .rotation = multiply(a->rotation, b->rotation)};
}
#ifdef __cplusplus
}
#endif
#endif //MYPROJECT_TRANSFORM_H