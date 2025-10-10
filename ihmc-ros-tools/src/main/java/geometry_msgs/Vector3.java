//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package geometry_msgs;

import org.ros.internal.message.Message;

public interface Vector3 extends Message {
   String _TYPE = "geometry_msgs/Vector3";
   String _DEFINITION = "# This represents a vector in free space. \n\nfloat64 x\nfloat64 y\nfloat64 z";

   double getX();

   void setX(double var1);

   double getY();

   void setY(double var1);

   double getZ();

   void setZ(double var1);
}
