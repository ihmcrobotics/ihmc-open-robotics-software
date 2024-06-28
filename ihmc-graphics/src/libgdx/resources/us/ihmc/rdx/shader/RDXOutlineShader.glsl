// Taken from https://github.com/mgsx-dev/gdx-gltf/tree/master/demo/core/src/net/mgsx/gltf/demo/shaders

#type vertex

attribute vec4 a_position;
attribute vec2 a_texCoord0;

uniform mat4 u_projTrans;
uniform vec2 u_size;

varying vec2 v_texCoords0;
varying vec2 v_texCoords1;
varying vec2 v_texCoords2;
varying vec2 v_texCoords3;
varying vec2 v_texCoords4;

void main()
{
    v_texCoords0 = a_texCoord0 + vec2(0.0, -1.0 / u_size.y);
    v_texCoords1 = a_texCoord0 + vec2(-1.0 / u_size.x, 0.0);
    v_texCoords2 = a_texCoord0;
    v_texCoords3 = a_texCoord0 + vec2(1.0 / u_size.x, 0.0);
    v_texCoords4 = a_texCoord0 + vec2(0.0, 1.0 / u_size.y);

    gl_Position = u_projTrans * a_position;
}

#type fragment

uniform sampler2D u_texture;

varying vec2 v_texCoords0;
varying vec2 v_texCoords1;
varying vec2 v_texCoords2;
varying vec2 v_texCoords3;
varying vec2 v_texCoords4;

uniform float u_depth_min;
uniform float u_depth_max;

uniform vec4 u_outer_color;
uniform vec4 u_inner_color;

//uniform float u_depthRange;

void main()
{
    const vec4 bitShifts = vec4(1.0, 1.0 / 255.0, 1.0 / 65025.0, 1.0 / 16581375.0);

    float depth = abs(
    dot(texture2D(u_texture, v_texCoords0), bitShifts) +
    dot(texture2D(u_texture, v_texCoords1), bitShifts) -
    dot(4.0 * texture2D(u_texture, v_texCoords2), bitShifts) +
    dot(texture2D(u_texture, v_texCoords3), bitShifts) +
    dot(texture2D(u_texture, v_texCoords4), bitShifts)
    );

    if (depth > u_depth_min)
    {
        if (depth < u_depth_max)
        {
            gl_FragColor = u_inner_color;
        }
        else
        {
            gl_FragColor = u_outer_color;
        }

//        float centerDepth = dot(texture2D(u_texture, v_texCoords2), bitShifts);
//        gl_FragColor.a *= 1.0 - pow(centerDepth, u_depthRange);
    }
    else
    {
        gl_FragColor = vec4(1.0, 1.0, 1.0, 0.0);
    }
}