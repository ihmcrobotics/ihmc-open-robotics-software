varying vec4 point;

void main()
{
  float l = length(point.xyz);
  
  gl_FragColor = vec4(l, 0, 0, 1.0);
}
