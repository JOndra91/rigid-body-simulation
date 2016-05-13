#include "DebugRenderer.hpp"

using namespace gmu;

DebugRenderer::DebugRenderer() {}
DebugRenderer::~DebugRenderer() {}

void DebugRenderer::SetPenColor( f32 r, f32 g, f32 b, f32 a ) {
    penColor = vec4(r, g, b, a);
}
void DebugRenderer::SetPenPosition( f32 x, f32 y, f32 z ) {
    if(lineActive) {
        lineIndicies.push_back(UINT32_MAX);
        lineActive = false;
    }
    penPosition = vec3(x,y,z);
}

void DebugRenderer::SetScale( f32 sx, f32 sy, f32 sz ) {
    // Not needed for BVH
}

// Render a line from pen position to this point.
// Sets the pen position to the new point.
void DebugRenderer::Line( f32 x, f32 y, f32 z ) {
    Vertex v = {
        .position = vec3(),
        .color = penColor
    };

    if(!lineActive) {
        v.position = penPosition;
        lineIndicies.push_back(lines.size());
        lines.push_back(v);

        lineActive = true;
    }

    v.position = vec3(x,y,z);
    lineIndicies.push_back(lines.size());
    lines.push_back(v);
}

void DebugRenderer::SetTriNormal( f32 x, f32 y, f32 z ) {
    // Not needed for BVH
}

// Render a triangle with the normal set by SetTriNormal.
void DebugRenderer::Triangle(
    f32 x1, f32 y1, f32 z1,
    f32 x2, f32 y2, f32 z2,
    f32 x3, f32 y3, f32 z3
) {
    // Not needed for BVH
}

// Draw a point with the scale from SetScale
void DebugRenderer::Point( ) {
    // Not needed for BVH
}
