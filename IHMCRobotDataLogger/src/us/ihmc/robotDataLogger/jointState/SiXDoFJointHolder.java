package us.ihmc.robotDataLogger.jointState;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;
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
      inverseDynamicsJoint.getRotation(rotation);
      inverseDynamicsJoint.getTranslation(translation);
      inverseDynamicsJoint.getJointTwist(twist);
      
      buffer[offset + 0]  = rotation.getW();
      buffer[offset + 1]  = rotation.getX();
      buffer[offset + 2]  = rotation.getY();
      buffer[offset + 3]  = rotation.getZ();
      
      buffer[offset + 4]  = translation.getX();
      buffer[offset + 5]  = translation.getY();
      buffer[offset + 6]  = translation.getZ();
      
      twist.getArray(buffer, offset + 7);
   }
}
