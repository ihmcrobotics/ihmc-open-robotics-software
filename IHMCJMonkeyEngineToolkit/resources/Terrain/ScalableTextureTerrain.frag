uniform sampler2D m_Texture;
uniform vec2 m_Scale;


varying vec2 texCoord;

#ifdef TRI_PLANAR_MAPPING
  varying vec4 vVertex;
  varying vec3 vNormal;
#else
	uniform float m_gridSize;  
#endif

void main(void)
{

#ifdef TRI_PLANAR_MAPPING
    // tri-planar texture bending factor for this fragment's normal
    vec3 blending = abs( vNormal );
    blending = (blending -0.2) * 0.7;
    blending = normalize(max(blending, 0.00001));      // Force weights to sum to 1.0 (very important!)
    float b = (blending.x + blending.y + blending.z);
    blending /= vec3(b, b, b);

    // texture coords
    vec4 coords = vVertex;

    vec4 col1 = texture2D( m_Texture, coords.yz * m_Scale.y);
    vec4 col2 = texture2D( m_Texture, coords.xz * m_Scale);
    vec4 col3 = texture2D( m_Texture, coords.xy * m_Scale.x);
    // blend the results of the 3 planar projections.
    vec4 texture = col1 * blending.x + col2 * blending.y + col3 * blending.z;
    
#else
	vec4 texture    = texture2D( m_Texture, texCoord.xy * m_Scale * m_gridSize ); // Tile
	
#endif

	gl_FragColor = texture;
}

