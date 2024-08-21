package geometry_msgs;

import org.ros.internal.message.Message;

public interface Quaternion extends Message {
   String _TYPE = "geometry_msgs/Quaternion";
   String _DEFINITION = "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";

   double getX();

   void setX(double var1);

   double getY();

   void setY(double var1);

   double getZ();

   void setZ(double var1);

   double getW();

   void setW(double var1);
}