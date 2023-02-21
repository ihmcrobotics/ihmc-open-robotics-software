#type vertex


in vec3 a_position;
uniform mat4 u_projViewTrans;


void main() {
   gl_Position = u_projViewTrans * vec4(a_position, 1.0);
}

void main() {
   uniform mat4 u_projTrans;
   uniform float u_fov;
   uniform float u_aspectRatio;
   uniform float u_near;
   uniform float u_far;

   void main() {
       // Calculate the yaw and pitch values based on the fragment position
       float x = gl_FragCoord.x;
       float y = gl_FragCoord.y;
       float num_cols = 128.0;  // Replace with the actual number of columns in your lidar scan
       float num_channels = 4.0;  // Replace with the actual number of channels in your lidar scan
       float yaw = (x / num_cols - 0.5) * 2.0 * PI;
       float pitch = (y / num_channels - 0.5) * u_fov / u_aspectRatio;

       // Create the transformation matrix for the current yaw and pitch values
       mat4 rotation = mat4(1.0);
       rotation = rotate(rotation, yaw, vec3(0.0, 1.0, 0.0));
       rotation = rotate(rotation, pitch, vec3(1.0, 0.0, 0.0));

       // Transform the vertex position to clip space
       gl_Position = u_projTrans * rotation * gl_Vertex;

       // Output the depth value to the color buffer
       gl_FragColor = vec4(2.0 * gl_FragCoord.z - 1.0, 0.0, 0.0, 1.0);

}


