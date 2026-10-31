---
sessionId: session-261028-173825-pp3p
---

# Requirements

### Overview
Wire the existing helper functions (`findReferenceEdge`, `findIncidentEdge`, `clipEdge`) into a `findContactPoints` function that generates 1–2 contact points for polygon-polygon collisions in `satPolygonPolygon`.

### Scope
- **In Scope**: Implement `findContactPoints` as a `static` helper in `SAT.c`, correct the three helper functions so they work with world-space polygons, and integrate the result into `SATres`.
- **Out of Scope**: Circle-polygon, circle-circle, manifold persistence, broad-phase, or impulse resolution.

### Functional Requirements
- Given two colliding polygons `A` and `B` and the minimum penetration axis (SAT normal), compute 1 or 2 contact points stored in `res.contacts[0..contactCount-1]`.
- Contact points must be in the same coordinate space as the polygon vertices (local space, since SAT runs on local vertices).
- The contact points must lie on or near the collision surface for stable impulse resolution.

# Technical Design

### Current Implementation

**SAT.c** contains three static helpers already written but unused:

1. **`findReferenceEdge(const Polygon* polygon, const vec2 normal)`** (lines 16–31) — finds the edge on a polygon whose normal most aligns with the given normal (maximum dot product). Returns an `Edge`.
2. **`findIncidentEdge(const Polygon* polygon, const vec2 referenceNormal)`** (lines 32–46) — finds the edge on a polygon whose normal is most opposing to the reference normal (minimum dot product). Returns an `Edge`.
3. **`clipEdge(const Edge* edge, vec2 normal, double offset, vec2 out[2])`** (lines 48–83) — clips an edge against a single side plane defined by `(normal, offset)`. Returns the number of clipped vertices (0, 1, or 2).

**Problem**: These functions operate on raw polygon vertices, but the contact point generation algorithm (from Erin Catto's Box2D-style clipping) requires the edges to be expressed *relative to the reference edge origin*, and the clipping normals need to be computed from the reference edge geometry.

### Key Decisions

- **Keep everything local-space**: Since `satPolygonPolygon` receives polygons with local-space vertices, contact points will also be in local space (relative to polygon A's origin). This avoids transform confusion.
- **Box2D-style clipping**: The algorithm is:
  1. Find reference edge on A (edge whose normal aligns most with the SAT normal)
  2. Find incident edge on B (edge whose normal opposes the SAT normal most)
  3. Translate incident edge vertices into reference edge's local frame
  4. Clip incident edge against reference edge's front plane (along the SAT normal)
  5. Clip the result against reference edge's two side planes (left and right along the edge)
  6. The remaining clipped vertices are the contact points

### Corrections to Existing Functions

#### 1. `findReferenceEdge` — needs correction

**Current issue**: It searches for the edge with the maximum dot product against the SAT normal. This is correct for finding the deepest edge, but the SAT normal direction might point into or out of the polygon depending on `normalOrient`. The reference edge should be the one whose normal is *most aligned* with the (already oriented) contact normal.

**Correction**: No logic change needed — the function is correct as-is. The SAT normal passed in has already been oriented by `normalOrient()` to point from A to B, so the reference edge on A should be the one facing that direction.

#### 2. `findIncidentEdge` — needs correction

**Current issue**: It finds the edge with the *minimum* dot product against the reference normal. This is correct — it finds the edge on B that faces most toward A.

**Correction**: No logic change needed.

#### 3. `clipEdge` — needs correction

**Current issue**: The function clips against a single plane, but the offset calculation isn't documented. The offset should be `dot(normal, referencePoint) - tolerance` where `referencePoint` is a point on the reference edge.

**Correction**: The function signature and logic are correct. The caller needs to compute the correct `normal` and `offset` for each clipping plane.

### Proposed Changes

#### Implement `findContactPoints` in SAT.c

```c
static void findContactPoints(const Polygon* a, const Polygon* b, const vec2 normal, SATres* res) {
    // Step 1: Find reference edge on A
    Edge refEdge = findReferenceEdge(a, normal);
    
    // Step 2: Find incident edge on B
    Edge incEdge = findIncidentEdge(b, scalarMul(normal, -1));
    
    // Step 3: Compute reference frame
    // Reference edge origin (midpoint or first vertex)
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
    if (frontCount < 2) {
        res->contactCount = 0;
        return;
    }
    
    // Step 6: Clip against side planes (tangent direction)
    // Left side plane: dot(v, refTangent) <= dot(refEdge.a, refTangent) + tolerance
    double leftOffset = dotProd(refEdge.a, refTangent) + 1e-9;
    vec2 sideClipped[2];
    size_t sideCount = clipEdge(
        &(Edge){frontClipped[0], frontClipped[1]},
        refTangent,
        leftOffset,
        sideClipped
    );
    if (sideCount < 2) {
        res->contactCount = 0;
        return;
    }
    
    // Right side plane: dot(v, -refTangent) <= dot(refEdge.b, -refTangent) + tolerance
    vec2 rightClipped[2];
    size_t rightCount = clipEdge(
        &(Edge){sideClipped[0], sideClipped[1]},
        scalarMul(refTangent, -1),
        dotProd(refEdge.b, scalarMul(refTangent, -1)) + 1e-9,
        rightClipped
    );
    if (rightCount < 2) {
        res->contactCount = 0;
        return;
    }
    
    // Step 7: Store contact points (transform back to world space)
    res->contacts[0] = add(rightClipped[0], refPoint);
    res->contactCount = 1;
    if (rightCount == 2) {
        res->contacts[1] = add(rightClipped[1], refPoint);
        res->contactCount = 2;
    }
}
```

### Architecture Diagram

```mermaid
graph LR
    A[satPolygonPolygon] --> B[Find min axis via SAT]
    B --> C[Orient normal with normalOrient]
    C --> D[findContactPoints]
    D --> E[findReferenceEdge on A]
    D --> F[findIncidentEdge on B]
    E --> G[Translate to ref frame]
    F --> G
    G --> H[clipEdge: front plane]
    H --> I[clipEdge: left side plane]
    I --> J[clipEdge: right side plane]
    J --> K[Store contacts in SATres]
```

### Risks
- **Zero-area clipping**: If the clipped edge degenerates to a point, `contactCount` should be 1, not 0. The current logic returns early on `< 2`, which might miss single-point contacts.
- **Tolerance tuning**: The `1e-9` epsilon in `clipEdge` might need adjustment for different scale ranges in the simulation.
- **Edge ordering**: The side-plane clipping order matters — left then right ensures the correct segment is retained.

# Delivery Steps

### ✓ Step 1: Outline findContactPoints algorithm and corrections
Produce a complete technical outline describing how `findContactPoints` wires together `findReferenceEdge`, `findIncidentEdge`, and `clipEdge`, including the necessary corrections to each helper.

- Document the Box2D-style clipping pipeline: reference edge selection → incident edge selection → coordinate transform → front plane clip → side plane clips → contact point extraction.
- Identify that `findReferenceEdge` and `findIncidentEdge` are logically correct but need the incident edge passed with a negated normal.
- Identify that `clipEdge` is correct but the caller must compute proper offsets for the three clipping planes (front, left side, right side).
- Provide the pseudocode/structure for `findContactPoints` showing the three clipping stages and how contact points are stored in `SATres`.

### ✓ Step 2: Implement findContactPoints function in SAT.c
Implement the `findContactPoints` static function in SAT.c following the outlined algorithm, wiring together the three existing helper functions with the correct coordinate transforms and clipping plane offsets.

### ✓ Step 3: Verify the implementation compiles and runs correctly
Build the project to ensure the new function compiles without errors and integrates properly with `satPolygonPolygon`.