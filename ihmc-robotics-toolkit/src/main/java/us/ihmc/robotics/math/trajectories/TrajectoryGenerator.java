package us.ihmc.robotics.math.trajectories;

public interface TrajectoryGenerator extends Finishable
{
   void initialize();

   void compute(double time);
}