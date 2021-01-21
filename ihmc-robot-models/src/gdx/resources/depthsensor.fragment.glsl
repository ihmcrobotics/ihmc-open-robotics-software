varying float v_depth;
varying vec4 point;

void main() {
	float depth = v_depth;
    const vec4 bias = vec4(1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0, 0.0);
    vec4 color = vec4(depth, fract(depth * 255.0), fract(depth * 65025.0), fract(depth * 16581375.0));
    gl_FragColor = color - (color.yzww * bias);
}
