//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package geometry_msgs;

import org.ros.internal.message.Message;

public interface Transform extends Message {
   String _TYPE = "geometry_msgs/Transform";
   String _DEFINITION = "# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n";

   Vector3 getTranslation();

   void setTranslation(Vector3 var1);

   Quaternion getRotation();

   void setRotation(Quaternion var1);
}
