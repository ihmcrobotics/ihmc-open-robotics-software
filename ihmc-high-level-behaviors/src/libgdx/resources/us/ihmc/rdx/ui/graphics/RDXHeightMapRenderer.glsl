#type vertex
#version 410

layout(location = 0) in float a_height;
layout(location = 1) in float a_traversability;

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;

uniform int u_centerIndex;
uniform vec2 u_gridCenter;
uniform float u_cellSize;
uniform int u_colorBasedOnTraversability;

float indexToCoordinate(int index, float gridCenter)
{
    return (index - u_centerIndex) * u_cellSize + gridCenter;
}

vec4 getColorTraversability()
{
    float t = a_traversability * a_traversability * a_traversability;
    if (a_traversability < 0.001)
        return vec4(0.82, 0.36, 0.28, 1.0);
    vec3 teal = vec3(0.18, 0.58, 0.62);
    vec3 gold = vec3(0.84, 0.62, 0.20);
    return vec4(mix(teal, gold, t), 1.0);
}

vec4 getColorFromHeight()
{
    // Muted but chromatic: teal, gold, coral, plum. No gray / white / black.
    float t = clamp((a_height + 0.15) / 1.10, 0.0, 1.0);
    vec3 teal = vec3(0.16, 0.58, 0.64);
    vec3 gold = vec3(0.84, 0.64, 0.18);
    vec3 coral = vec3(0.86, 0.40, 0.30);
    vec3 plum = vec3(0.64, 0.30, 0.70);
    vec3 rgb;
    if (t < 0.33)
        rgb = mix(teal, gold, t / 0.33);
    else if (t < 0.66)
        rgb = mix(gold, coral, (t - 0.33) / 0.33);
    else
        rgb = mix(coral, plum, (t - 0.66) / 0.34);
    return vec4(rgb, 1.0);
}

void main()
{
    int cellsPerAxis = 2 * u_centerIndex + 1; // width and height of the height map

    int xIndex = gl_VertexID / cellsPerAxis;
    int yIndex = gl_VertexID % cellsPerAxis;

    float xPosition = indexToCoordinate(xIndex, u_gridCenter.x);
    float yPosition = indexToCoordinate(yIndex, u_gridCenter.y);
    float zPosition = a_height;

	vec4 pointInCameraFrame = u_viewTrans * vec4(xPosition, yPosition, zPosition, 1);
	vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);

	vec4 projectedSpriteCorner = u_projTrans * vec4(u_cellSize, u_cellSize, pointInCameraFrame.z, pointInCameraFrame.w);
	float projectedSize = u_screenWidth * projectedSpriteCorner.x / projectedSpriteCorner.w;

	gl_PointSize = 0.34 * (projectedSize);

	gl_Position = u_projTrans * pointInCameraFrame;

    if (u_colorBasedOnTraversability > 0)
    	v_color = getColorTraversability();
    else
    	v_color = getColorFromHeight();
}

#type fragment
#version 410

in vec4 v_color;
out vec4 color;

void main()
{
	color = v_color;
}
