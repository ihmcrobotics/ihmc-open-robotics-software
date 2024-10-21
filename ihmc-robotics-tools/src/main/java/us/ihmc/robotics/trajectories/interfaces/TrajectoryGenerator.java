package us.ihmc.robotics.trajectories.interfaces;

public interface TrajectoryGenerator extends Finishable
{
   void initialize();

   void compute(double time);
}