#pragma once

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <q3.h>

namespace gmu {

    using glm::vec3;
    using glm::vec4;
    using std::vector;
    class DebugRenderer : public q3Render {
    public:
        struct Vertex {
            vec3 position;
            vec4 color;
        };

        vec4 penColor;
        vec3 penPosition;
        vector<Vertex> lines;
        vector<GLuint> lineIndicies;
        bool lineActive = false;

        DebugRenderer();
        ~DebugRenderer();

        virtual void SetPenColor( f32 r, f32 g, f32 b, f32 a = 1.0f ) override;
        virtual void SetPenPosition( f32 x, f32 y, f32 z ) override;
        virtual void SetScale( f32 sx, f32 sy, f32 sz ) override;

        // Render a line from pen position to this point.
        // Sets the pen position to the new point.
        virtual void Line( f32 x, f32 y, f32 z ) override;

        virtual void SetTriNormal( f32 x, f32 y, f32 z ) override;

        // Render a triangle with the normal set by SetTriNormal.
        virtual void Triangle(
            f32 x1, f32 y1, f32 z1,
            f32 x2, f32 y2, f32 z2,
            f32 x3, f32 y3, f32 z3
        ) override;

        // Draw a point with the scale from SetScale
        virtual void Point( ) override;
    };
}
