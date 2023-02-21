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

void main() {

    vec3 lightPosition = vec3(0.0, 0.0, 0.0);

    vec3 lightDirection = normalize(lightPosition - gl_FragCoord.xyz);

    out_color = vec4(0.0, 1.0, 0.0, 1.0);
}


