package geometry_msgs;

import org.ros.internal.message.Message;

public interface Point extends Message {
   String _TYPE = "geometry_msgs/Point";
   String _DEFINITION = "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";

   double getX();

   void setX(double var1);

   double getY();

   void setY(double var1);

   double getZ();

   void setZ(double var1);
}