package us.ihmc.commonWalkingControlModules.configurations;

public abstract class ICPPlannerParameters implements ICPTrajectoryPlannerParameters, CoPPlannerParameters
{
   /**
    * <p>
    * Sets which ICP planner to use.
    * </p>
    * <p>
    * If true, uses the new {@link us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.SmoothCMPBasedICPPlanner}.
    * If false, uses the traditional {@link us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ContinuousCMPBasedICPPlanner}.
    * </p>
    */
   public abstract boolean useSmoothCMPPlanner();
}
