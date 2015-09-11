package us.ihmc.robotics.math.trajectories;

public interface TrajectoryGenerator extends Finishable
{
   public abstract void initialize();

   public abstract void compute(double time);
}