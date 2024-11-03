package us.ihmc.robotics.math.trajectories.interfaces;

public interface TrajectoryGenerator extends Finishable
{
   void initialize();

   void compute(double time);
}