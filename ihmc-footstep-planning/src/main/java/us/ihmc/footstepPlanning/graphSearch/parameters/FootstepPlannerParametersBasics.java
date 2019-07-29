package us.ihmc.footstepPlanning.graphSearch.parameters;

public interface FootstepPlannerParametersBasics extends FootstepPlannerParametersReadOnly
{
   default void set(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      getStoredPropertySet().setAll(footstepPlannerParameters.getStoredPropertySet().getAll());
   }

   default void setMinimumDistanceFromCliffBottoms(double distance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minimumDistanceFromCliffBottoms, distance);
   }

   default void setCliffHeightToAvoid(double height)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.cliffHeightToAvoid, height);
   }

   default void setMaximumStepReach(double reach)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepReach, reach);
   }

   default void setMaximumStepWidth(double width)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepWidth, width);
   }

   default void setMaximumStepYaw(double yaw)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepYaw, yaw);
   }

   default void setMinimumStepYaw(double yaw)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minStepYaw, yaw);
   }

   default void setMaximumStepZ(double stepZ)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepZ, stepZ);
   }

   default void setMaximumXYWiggleDistance(double wiggleDistance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumXYWiggleDistance, wiggleDistance);
   }

   default void setMaximumYawWiggle(double wiggleDistance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumYawWiggle, wiggleDistance);
   }

   default void setMinimumFootholdPercent(double footholdPercent)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minFootholdPercent, footholdPercent);
   }

   default void setMinimumStepLength(double minimumStepLength)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minStepLength, minimumStepLength);
   }

   default void setMinimumStepWidth(double minimumStepWidth)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minStepWidth, minimumStepWidth);
   }

   default void setMinimumSurfaceInclineRadians(double surfaceInclineRadians)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minSurfaceIncline, surfaceInclineRadians);
   }

   default void setMinXClearanceFromStance(double clearance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minXClearanceFromStance, clearance);
   }

   default void setMinYClearanceFromStance(double clearance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minYClearanceFromStance, clearance);
   }

   default void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.wiggleInsideDelta, wiggleInsideDelta);
   }

   default void setMaximumStepZWhenSteppingUp(double maxStepZ)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepZWhenSteppingUp, maxStepZ);
   }

   default void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   default void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   default void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }
}