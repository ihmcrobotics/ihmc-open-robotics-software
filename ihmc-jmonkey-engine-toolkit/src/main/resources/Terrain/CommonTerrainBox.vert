uniform mat4 g_WorldViewProjectionMatrix;
uniform float m_gridSize;

attribute vec3 inPosition;
attribute vec2 inTexCoord;

varying vec4 vVertex;

void main(){
    gl_Position = g_WorldViewProjectionMatrix * vec4(inPosition, 1.0);
	vVertex = vec4(inPosition.x, inPosition.z, inPosition.y, 0.0);
}