package geometry_msgs;

import org.ros.internal.message.Message;

public interface Pose extends Message {
   String _TYPE = "geometry_msgs/Pose";
   String _DEFINITION = "# A representation of pose in free space, composed of postion and orientation. \nPoint position\nQuaternion orientation\n";

   Point getPosition();

   void setPosition(Point var1);

   Quaternion getOrientation();

   void setOrientation(Quaternion var1);
}