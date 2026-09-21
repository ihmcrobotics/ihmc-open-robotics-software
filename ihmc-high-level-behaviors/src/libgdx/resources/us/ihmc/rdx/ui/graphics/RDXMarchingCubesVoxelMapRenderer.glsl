#type vertex
#version 410

layout(location = 0) in vec3 a_position; // world-space triangle vertex

out vec3 v_worldPos;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;

void main()
{
    v_worldPos = a_position;
    gl_Position = u_projTrans * u_viewTrans * vec4(a_position, 1.0);
}

#type fragment
#version 410

in vec3 v_worldPos;
out vec4 color;

void main()
{
    // Flat per-triangle normal from screen-space derivatives — no normal VBO needed.
    vec3 dx = dFdx(v_worldPos);
    vec3 dy = dFdy(v_worldPos);
    vec3 normal = normalize(cross(dx, dy));

    // Two-sided diffuse + ambient so interior faces are still lit.
    vec3 sunDir = normalize(vec3(0.4, 0.3, 1.0));
    float diffuse = abs(dot(normal, sunDir));
    float brightness = 0.30 + 0.70 * diffuse;

    // Height-based hue: cool blue near floor, warm orange for elevated obstacles.
    // World Z in IHMC convention: 0 = pelvis height, negative = below.
    float h = clamp((v_worldPos.z + 1.2) / 1.8, 0.0, 1.0); // map [-1.2, 0.6] → [0, 1]
    vec3 floorColor    = vec3(0.30, 0.55, 0.80); // cool blue
    vec3 midColor      = vec3(0.40, 0.80, 0.45); // green
    vec3 obstColor     = vec3(0.90, 0.55, 0.20); // warm orange

    vec3 baseColor;
    if (h < 0.5)
        baseColor = mix(floorColor, midColor, h * 2.0);
    else
        baseColor = mix(midColor, obstColor, (h - 0.5) * 2.0);

    color = vec4(baseColor * brightness, 1.0);
}
