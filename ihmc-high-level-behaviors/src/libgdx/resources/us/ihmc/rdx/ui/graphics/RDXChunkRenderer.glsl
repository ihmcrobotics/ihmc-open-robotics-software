#type vertex
#version 410

layout(location = 0) in float a_height;

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;

uniform int u_cellsPerAxis;
uniform vec2 u_chunkOrigin;
uniform float u_cellSize;

float linearInterpolate(float a, float b, float alpha)
{
    return (1.0f - alpha) * a + alpha * b;
}

vec4 getColor(float height)
{
    float r = 0, g = 0, b = 0;
    float magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
    float orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
    float yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
    float blueR = 0.0, blueG = 0.0, blueB = 1.0;
    float greenR = 0.0, greenG = 1.0, greenB = 0.0;
    float gradientSize = 0.2;
    float gradientLength = 1.0;
    float alpha = mod(height, gradientLength);
    if (alpha < 0)
        alpha = 1 + alpha;
    while (alpha > 5 * gradientSize)
        alpha -= 5 * gradientSize;

    if (alpha <= gradientSize * 1)
    {
        r = linearInterpolate(magentaR, blueR, (alpha) / gradientSize);
        g = linearInterpolate(magentaG, blueG, (alpha) / gradientSize);
        b = linearInterpolate(magentaB, blueB, (alpha) / gradientSize);
    }
    else if (alpha <= gradientSize * 2)
    {
        r = linearInterpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
        g = linearInterpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
        b = linearInterpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
    }
    else if (alpha <= gradientSize * 3)
    {
        r = linearInterpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
        g = linearInterpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
        b = linearInterpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
    }
    else if (alpha <= gradientSize * 4)
    {
        r = linearInterpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
        g = linearInterpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
        b = linearInterpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
    }
    else if (alpha <= gradientSize * 5)
    {
        r = linearInterpolate(orangeR, magentaR, (alpha - gradientSize * 4) / gradientSize);
        g = linearInterpolate(orangeG, magentaG, (alpha - gradientSize * 4) / gradientSize);
        b = linearInterpolate(orangeB, magentaB, (alpha - gradientSize * 4) / gradientSize);
    }

    return vec4(r, g, b, 1.0f);
}

void main()
{
    int xIndex = gl_VertexID / u_cellsPerAxis;
    int yIndex = gl_VertexID % u_cellsPerAxis;

    float xPosition = u_chunkOrigin.x + float(xIndex) * u_cellSize;
    float yPosition = u_chunkOrigin.y + float(yIndex) * u_cellSize;
    float zPosition = a_height;

    vec4 pointInCameraFrame = u_viewTrans * vec4(xPosition, yPosition, zPosition, 1);
    vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);

    vec4 projectedSpriteCorner = u_projTrans * vec4(u_cellSize, u_cellSize, pointInCameraFrame.z, pointInCameraFrame.w);
    float projectedSize = u_screenWidth * projectedSpriteCorner.x / projectedSpriteCorner.w;

    gl_PointSize = 0.5 * (projectedSize);
    gl_Position = u_projTrans * pointInCameraFrame;

    v_color = getColor(zPosition);
}

#type fragment
#version 410

in vec4 v_color;
out vec4 color;

void main()
{
	color = v_color;
}
