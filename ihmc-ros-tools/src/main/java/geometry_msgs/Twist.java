package geometry_msgs;

import org.ros.internal.message.Message;

public interface Twist extends Message {
   String _TYPE = "geometry_msgs/Twist";
   String _DEFINITION = "# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n";

   Vector3 getLinear();

   void setLinear(Vector3 var1);

   Vector3 getAngular();

   void setAngular(Vector3 var1);
}
