package us.ihmc.robotics.math.trajectories.interfaces;

public interface OneDoFJointTrajectoryGenerator extends DoubleTrajectoryGenerator
{
   public abstract void initialize(double initialPosition, double initialVelocity);
}
