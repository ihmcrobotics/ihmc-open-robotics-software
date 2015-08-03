package us.ihmc.robotDataCommunication.jointState;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;

public class SiXDoFJointHolder implements JointHolder
{
   private final SixDoFJoint inverseDynamicsJoint;
   
   private final Quat4d rotation = new Quat4d();
   private final Vector3d translation = new Vector3d();
   private final Twist twist = new Twist();
   
   public SiXDoFJointHolder(SixDoFJoint joint)
   {
      this.inverseDynamicsJoint = joint;
   }

   public JointType getJointType()
   {
      return JointType.SiXDoFJoint;
   }

   public int getNumberOfStateVariables()
   {
      return 13;  // quaternion + position + angular velocity + linear velocity
   }

   public void get(double[] buffer, int offset)
   {
      inverseDynamicsJoint.packRotation(rotation);
      inverseDynamicsJoint.packTranslation(translation);
      inverseDynamicsJoint.packJointTwist(twist);
      
      buffer[offset + 0]  = rotation.w;
      buffer[offset + 1]  = rotation.x;
      buffer[offset + 2]  = rotation.y;
      buffer[offset + 3]  = rotation.z;
      
      buffer[offset + 4]  = translation.x;
      buffer[offset + 5]  = translation.y;
      buffer[offset + 6]  = translation.z;
      
      twist.packArray(buffer, offset + 7);
   }
}
