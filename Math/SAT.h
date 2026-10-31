//
// Created by thong on 31/07/2026.
//

#ifndef UNTITLED_SAT_H
#define UNTITLED_SAT_H
#pragma once
#include "polygon.h"
#ifdef __cplusplus
extern "C"{
#endif
typedef struct {
    bool collided;
    vec2 normal;
    double depth;
    vec2 contacts[2];
    size_t contactCount;
}SATres;
SATres satPolygonPolygon(const Polygon* a, const Polygon* b);
#ifdef __cplusplus
}
#endif
#endif //UNTITLED_SAT_H