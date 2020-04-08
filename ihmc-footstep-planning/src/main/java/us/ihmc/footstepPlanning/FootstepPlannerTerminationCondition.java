package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;

public interface FootstepPlannerTerminationCondition
{
   /**
    * Planner termination conditions are checked after each iteration (~30Hz), so they should be lightweight.
    * If true, the planner returns the least cost plan immediately.
    *
    * @param plannerTime total time taken by planner
    * @param iterations number of iterations
    * @param bestPathFinalStep final step of the current best path
    * @param bestPathSize number of steps to get to bestPathFinalStep
    * @return whether to terimate the planner
    */
   boolean terminatePlanner(double plannerTime, int iterations, Pose2DReadOnly bestPathFinalStep, int bestPathSize);
}
