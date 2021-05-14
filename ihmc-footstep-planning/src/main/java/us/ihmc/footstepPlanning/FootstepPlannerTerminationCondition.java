package us.ihmc.footstepPlanning;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface FootstepPlannerTerminationCondition
{
   /**
    * Planner termination conditions are checked after each iteration (~30Hz), so they should be lightweight.
    * If true, the planner returns the least cost plan immediately.
    *
    * @param plannerTime total time taken by planner
    * @param iterations number of iterations
    * @param bestFinalStep final step of the current best path
    * @param bestSecondToLastStep second to last step of the current best path
    * @param bestPathSize number of steps to get to bestPathFinalStep
    * @return whether to terimate the planner
    */
   boolean terminatePlanner(double plannerTime, int iterations, RigidBodyTransformReadOnly bestFinalStep, RigidBodyTransformReadOnly bestSecondToLastStep, int bestPathSize);
}
