package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface InverseKinematicsCalculator
{
   public abstract boolean solve(RigidBodyTransformReadOnly desiredTransform);
   
   public abstract double getErrorScalar();
   
   public abstract int getNumberOfIterations();

   public abstract void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener);

   public abstract void setLimitJointAngles(boolean limitJointAngles);
}
