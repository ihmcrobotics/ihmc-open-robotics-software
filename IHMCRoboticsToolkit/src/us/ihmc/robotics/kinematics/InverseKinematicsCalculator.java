package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface InverseKinematicsCalculator
{
   public abstract boolean solve(RigidBodyTransform desiredTransform);
   
   public abstract double getErrorScalar();
   
   public abstract int getNumberOfIterations();

   public abstract void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener);

   public abstract void setLimitJointAngles(boolean limitJointAngles);
}
