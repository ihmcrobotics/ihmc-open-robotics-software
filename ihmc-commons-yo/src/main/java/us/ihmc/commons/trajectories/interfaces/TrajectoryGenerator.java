package us.ihmc.commons.trajectories.interfaces;

public interface TrajectoryGenerator extends Finishable
{
   void initialize();

   void compute(double time);
}