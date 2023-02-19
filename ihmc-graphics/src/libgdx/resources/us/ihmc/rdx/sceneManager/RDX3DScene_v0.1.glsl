#type vertex

in vec3 a_position;
uniform mat4 u_projViewTrans;
uniform mat4 u_worldTrans;

void main() {

    vec4 pos = u_worldTrans * vec4(a_position, 1.0);

    gl_Position = u_projViewTrans * pos;

}

#type fragment

out vec4 out_color;
out float out_processedDepth;

void main() {

    out_color.a = 1.0;
    out_processedDepth = 2.0 * gl_FragCoord.z - 1.0; // Normalized to -1.0 to 1.0
}


