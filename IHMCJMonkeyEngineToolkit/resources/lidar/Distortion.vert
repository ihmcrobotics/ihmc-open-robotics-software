uniform mat4 g_WorldViewProjectionMatrix;
uniform mat4 g_WorldViewMatrix;
uniform mat4 g_ProjectionMatrix;

varying vec2 texCoord1;
attribute vec2 inTexCoord;
attribute vec3 inPosition;

void main()
{
  gl_Position = g_WorldViewProjectionMatrix * vec4(inPosition, 1.0);
  texCoord1.xy = inTexCoord.xy;
}