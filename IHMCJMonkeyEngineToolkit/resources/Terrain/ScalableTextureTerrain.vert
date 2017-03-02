uniform mat4 g_WorldViewProjectionMatrix;

attribute vec3 inPosition;
attribute vec3 inNormal;
attribute vec2 inTexCoord;

varying vec2 texCoord;

#ifdef TRI_PLANAR_MAPPING
  varying vec4 vVertex;
  varying vec3 vNormal;
  uniform float m_gridSize;
#endif

void main(){
    gl_Position = g_WorldViewProjectionMatrix * vec4(inPosition, 1.0);
    texCoord = inTexCoord;

#ifdef TRI_PLANAR_MAPPING
    vVertex = vec4(texCoord.x * m_gridSize, inPosition.y, texCoord.y * m_gridSize, 0.0);
    vNormal = inNormal;
#endif

}