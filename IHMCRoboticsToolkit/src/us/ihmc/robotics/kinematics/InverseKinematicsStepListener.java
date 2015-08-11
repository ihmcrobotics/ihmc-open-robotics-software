package us.ihmc.robotics.kinematics;

public interface InverseKinematicsStepListener
{
   public abstract void didAnInverseKinemticsStep(double errorScalar);
}
