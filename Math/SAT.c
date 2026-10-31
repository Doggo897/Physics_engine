//
// Created by thong on 03/08/2026.
//
#include <stddef.h>
#include "SAT.h"
#include "vec2.h"
#include <math.h>
typedef struct {
    double min;
    double max;
}Projection;
typedef struct {
	vec2 a;
	vec2 b;
}Edge;
static Edge findReferenceEdge(const Polygon* polygon, const vec2 normal) {
	Edge res = {};
	double max = -INFINITY;
	double d = 0;
	for(size_t i = 0; i < polygon->count; ++i) {
		vec2 edgeNormal = polygonEdgeNormal(polygon, i);
		d = dotProd(edgeNormal, normal);
		if(d > max){
			max = d;
			res.a = polygon->vertices[i];
			res.b = polygon->vertices[(i+1)%polygon->count];
		}
	}
	return res;
}
static Edge findIncidentEdge(const Polygon* polygon, const vec2 referenceNormal) {
	Edge res = {};
	double min = INFINITY;
	for(size_t i = 0; i < polygon->count; ++i) {
		vec2 edgeNormal = polygonEdgeNormal(polygon, i);
		double d = dotProd(edgeNormal, referenceNormal);
		if(d < min){
			min = d;
			res.a = polygon->vertices[i];
			res.b = polygon->vertices[(i+1)%polygon->count];
		}
	}
	return res;
}
static size_t clipEdge(const Edge* edge, vec2 normal, double offset, vec2 out[2]) {
	const double eps = 1e-9;
	const vec2 a = edge->a;
	const vec2 b = edge->b;
	const double da = dotProd(normal, a) - offset;
	const double db = dotProd(normal, b) - offset;
	const bool inA = da <= eps;
	const bool inB = db <= eps;
	if (inA && inB){
		out[0] = a;
		out[1] = b;
		return 2;
	}
	if (!inA && !inB){
		return 0;
	}
	if (fabs(da) <= eps){
		out[0] = a;
		return 1;
	}
	if (fabs(db) <= eps){
		out[0] = b;
		return 1;
	}
	const double t = da / (da - db);
	const vec2 intersection = add(a,scalarMul(sub(b, a), t));
	if (inA){
		out[0] = a;
		out[1] = intersection;
	}
	else{
		out[0] = intersection;
		out[1] = b;
	}
	return 2;
}
static Projection projectPolygon(const Polygon* polygon, const vec2 axis) {
    Projection p;
    double projection = dotProd(polygon->vertices[0], axis);
    p.min = projection;
    p.max = projection;
    for (size_t i = 1; i < polygon->count; i++) {
        projection = dotProd(polygon->vertices[i], axis);
        if (projection < p.min) p.min = projection;
        if (projection > p.max) p.max = projection;
    }
    return p;
}
static bool overlap(const Projection a, const Projection b) {
    return !(a.max<b.min||b.max<a.min);
}
static double overlapDepth(const Projection a,const Projection b){
    return fmin(a.max, b.max) - fmax(a.min, b.min);
}
static vec2 normalOrient (vec2 axis, const Polygon* a, const Polygon* b) {
    if (dotProd(sub(polygonCentroid(b),polygonCentroid(a)), axis) < 0)
    {
        axis = scalarMul(axis, -1);
    }
    return axis;
}
static void findContactPoints(const Polygon* a, const Polygon* b, const vec2 normal, SATres* res) {
    // Step 1: Find reference edge on A
    Edge refEdge = findReferenceEdge(a, normal);

    // Step 2: Find incident edge on B
    Edge incEdge = findIncidentEdge(b, scalarMul(normal, -1.0));

    // Step 3: Compute reference frame
    // Reference edge origin (first vertex of reference edge)
    vec2 refPoint = refEdge.a;

    // Reference edge direction (tangent)
    vec2 refTangent = normalize(sub(refEdge.b, refEdge.a));

    // Reference normal (should align with SAT normal)
    vec2 refNormal = normal;

    // Step 4: Translate incident edge to reference frame
    vec2 incV0 = sub(incEdge.a, refPoint);
    vec2 incV1 = sub(incEdge.b, refPoint);

    // Step 5: Clip against front plane (reference normal)
    // Front plane: dot(v, refNormal) <= 0 (behind the reference edge)
    vec2 frontClipped[2];
    size_t frontCount = clipEdge(
        &(Edge){incV0, incV1},
        refNormal,
        0.0,  // offset: plane through refPoint
        frontClipped
    );
    if (frontCount < 1) {
        res->contactCount = 0;
        return;
    }

    // Step 6: Clip against side planes (tangent direction)
    // Left side plane: dot(v, refTangent) <= dot(refEdge.a, refTangent) + tolerance
    // Note: refEdge.a is the reference origin, so dot(refEdge.a, refTangent) = 0
    double leftOffset = 0.0 + 1e-9;
    vec2 sideClipped[2];
    size_t sideCount = clipEdge(
        &(Edge){frontClipped[0], frontClipped[1]},
        refTangent,
        leftOffset,
        sideClipped
    );
    if (sideCount < 1) {
        res->contactCount = 0;
        return;
    }

    // Right side plane: dot(v, -refTangent) <= dot(refEdge.b - refEdge.a, -refTangent) + tolerance
    vec2 rightClipped[2];
    size_t rightCount = clipEdge(
        &(Edge){sideClipped[0], sideClipped[1]},
        scalarMul(refTangent, -1.0),
        0.0 + 1e-9,
        rightClipped
    );
    if (rightCount < 1) {
        res->contactCount = 0;
        return;
    }

    // Step 7: Store contact points (transform back to original space)
    res->contacts[0] = add(rightClipped[0], refPoint);
    res->contactCount = 1;
    if (rightCount == 2) {
        res->contacts[1] = add(rightClipped[1], refPoint);
        res->contactCount = 2;
    }
    
    // Compute contact depth as the average penetration along the normal
    if (res->contactCount > 0) {
        double totalDepth = 0.0;
        for (size_t i = 0; i < res->contactCount; i++) {
            totalDepth += -dotProd(sub(res->contacts[i], refPoint), refNormal);
        }
        res->depth = totalDepth / res->contactCount;
    }
}

SATres satPolygonPolygon(const Polygon* a, const Polygon* b) {
    SATres res = {
        .collided = false,
        .normal = (vec2){0, 0},
        .depth = 0,
        .contacts = {{0, 0}, {0, 0}},
        .contactCount = 0
    };
    double minOverlap = INFINITY;
    vec2 minAxis = (vec2){0, 0};
    double depth = 0;
    Projection projA;
    Projection projB;
    for (size_t i = 0; i < a->count; ++i) {
        const vec2 axis = polygonEdgeNormal(a, i);
        projA = projectPolygon(a, axis);
        projB = projectPolygon(b, axis);
        if (!overlap(projA, projB)) {
            return res;
        }
        depth = overlapDepth(projA, projB);
        if (depth < minOverlap) {
            minOverlap = depth;
            minAxis = axis;
        }
    }
    for (size_t i = 0; i < b->count; ++i) {
        const vec2 axis = polygonEdgeNormal(b, i);
        projA = projectPolygon(a, axis);
        projB = projectPolygon(b, axis);
        if (!overlap(projA, projB)) {
            return res;
        }
        depth = overlapDepth(projA, projB);
        if (depth < minOverlap) {
            minOverlap = depth;
            minAxis = axis;
        }
    }
    minAxis = normalOrient(minAxis, a, b);
    res.collided = true;
    res.normal = minAxis;
    res.depth = minOverlap;
    findContactPoints(a, b, minAxis, &res);
    return res;
}