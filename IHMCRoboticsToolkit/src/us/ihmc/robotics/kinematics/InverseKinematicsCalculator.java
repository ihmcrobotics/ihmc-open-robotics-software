package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;

public interface InverseKinematicsCalculator
{
   public abstract boolean solve(RigidBodyTransform desiredTransform);
   
   public abstract double getErrorScalar();
   
   public abstract int getNumberOfIterations();

   public abstract void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener);

   public abstract void setLimitJointAngles(boolean limitJointAngles);
}
