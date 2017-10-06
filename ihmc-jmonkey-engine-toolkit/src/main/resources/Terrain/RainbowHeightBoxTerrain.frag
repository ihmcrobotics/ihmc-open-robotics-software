#define pi 3.141592653589793238462643383279

uniform float m_minHeight;
uniform float m_maxHeight;
uniform vec2 m_scale;
uniform vec3 m_lineColor; 
uniform float m_alpha;
uniform float m_lineWidth;
uniform float m_radius;

varying vec4 vVertex;


void main(void)
{
	float r;
	float g;
	float b;
	float a;
	

	float xLine = mod(vVertex.x * m_scale.x, 1.0);
	float zLine = mod(vVertex.z * m_scale.y, 1.0);
	
	if ( xLine > (1.0 - m_lineWidth/2.0) || xLine < m_lineWidth/2.0 || zLine > (1.0 - m_lineWidth/2.0) || zLine < m_lineWidth/2.0)
	{
		r = m_lineColor.x;
		g = m_lineColor.y;
		b = m_lineColor.z;
		a = 1.0;
	}
	else
	{
		float index = (vVertex.y + m_minHeight)/(m_maxHeight - m_minHeight) * 2.0 * pi; 
		r = sin(index + 0.66) * m_radius+ m_radius;
		g = sin(index + 0.0 * pi) * m_radius+ m_radius;
		b = sin(index + 1.33 * pi) * m_radius+ m_radius;
		a = m_alpha;
	}
	
	
	gl_FragColor = vec4(r, g, b, a);
}


