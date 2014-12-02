uniform sampler2D m_tex0;
uniform sampler2D m_tex1;
uniform sampler2D m_tex2;
uniform sampler2D m_tex3;

varying vec2 texCoord1;

uniform float m_startAngle;
uniform float m_step;
uniform float m_resolution;

uniform float m_camModulo;
uniform float m_camModuloPostOffset;
uniform float m_camModuloPreOffset;
uniform float m_oneOverCameras;

uniform float m_cameraAngleToTextureAngle;

void main()
{
	// Calculate the scan index
  	float p = (texCoord1.x - m_resolution / 2.0) / (1.0 - m_resolution);
  
    // Find the angle of the corresponding scan ray
  	float scanAngle = m_startAngle + m_step * p;  	
  	
  	// Modulo the angle to account for multiple cameras. 
	float textureAngle = mod(scanAngle + m_camModuloPreOffset, m_camModulo) + m_camModuloPostOffset;

	// Find the new point on the scan texture for this ray
  	vec2 point = vec2(0.5 * tan(textureAngle) * m_cameraAngleToTextureAngle + 0.5, texCoord1.y);

	// Find texture to use for this ray
	float tex = floor(texCoord1.x / m_oneOverCameras);

	// Set the correct depth for this texture
	if (tex == 0.0)
	{
			gl_FragColor = texture2D(m_tex0, point); 	
	}
	if (tex == 1.0)
	{
			gl_FragColor = texture2D(m_tex1, point); 	
	}				
	if (tex == 2.0)
	{
			gl_FragColor = texture2D(m_tex2, point);
	}
	if (tex == 3.0)
	{
			gl_FragColor = texture2D(m_tex3, point);
	}
	
	
	// Use for debugging, set the green, blue, and alpha value to debug statements. 
	gl_FragColor = vec4(gl_FragColor.r, point.x, textureAngle, tex);
}