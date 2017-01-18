uniform mat4 g_WorldViewProjectionMatrix;
uniform mat4 g_WorldViewMatrix;
uniform mat4 g_ProjectionMatrix;

attribute vec3 inPosition;

varying vec4 point;


void main()
{
  gl_Position = g_WorldViewProjectionMatrix * vec4(inPosition, 1.0);

  // Vertex in world space
   point = g_WorldViewMatrix * vec4(inPosition, 1.0);   
}