package us.ihmc.robotDataLogger.jointState;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotDataLogger.JointType;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class SiXDoFJointHolder implements JointHolder
{
   private final SixDoFJoint inverseDynamicsJoint;
   

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
      QuaternionReadOnly rotation = inverseDynamicsJoint.getRotationForReading();
      Tuple3DReadOnly translation = inverseDynamicsJoint.getTranslationForReading();
      Tuple3DReadOnly angularVelocity = inverseDynamicsJoint.getAngularVelocityForReading();
      Tuple3DReadOnly linearVelocity = inverseDynamicsJoint.getLinearVelocityForReading();

      buffer[offset++] = rotation.getS();
      buffer[offset++] = rotation.getX();
      buffer[offset++] = rotation.getY();
      buffer[offset++] = rotation.getZ();

      buffer[offset++] = translation.getX();
      buffer[offset++] = translation.getY();
      buffer[offset++] = translation.getZ();

      buffer[offset++] = angularVelocity.getX();
      buffer[offset++] = angularVelocity.getY();
      buffer[offset++] = angularVelocity.getZ();

      buffer[offset++] = linearVelocity.getX();
      buffer[offset++] = linearVelocity.getY();
      buffer[offset] = linearVelocity.getZ();
   }


   @Override
   public String getName()
   {
      return inverseDynamicsJoint.getName();
   }
}
