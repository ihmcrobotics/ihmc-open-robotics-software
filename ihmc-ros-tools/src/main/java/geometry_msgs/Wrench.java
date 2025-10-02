package geometry_msgs;

import org.ros.internal.message.Message;

public interface Wrench extends Message {
   String _TYPE = "geometry_msgs/Wrench";
   String _DEFINITION = "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n";

   Vector3 getForce();

   void setForce(Vector3 var1);

   Vector3 getTorque();

   void setTorque(Vector3 var1);
}