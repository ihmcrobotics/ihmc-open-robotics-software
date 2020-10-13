package us.ihmc.footstepPlanning.swing;

import controller_msgs.msg.dds.SwingPlannerParametersPacket;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

/**
 * Parameters for swing planning as part of the planning pipeline in {@link FootstepPlanningModule}
 */
public interface SwingPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Collision boxes are checked for the start and end of swing at the toe and heel, respectively.
    * If a collision is detected at either, this swing height is used.
    */
   default double getSwingHeightIfCollisionDetected()
   {
      return get(SwingPlannerParameterKeys.swingHeightIfCollisionDetected);
   }

   /**
    * Specifies the minimum swing time in the swing waypoint proportion calculator.
    */
   default double getMinimumSwingTime()
   {
      return get(SwingPlannerParameterKeys.minimumSwingTime);
   }

   /**
    * Specifies the maximum swing time in the swing waypoint proportion calculator
    */
   default double getMaximumSwingTime()
   {
      return get(SwingPlannerParameterKeys.maximumSwingTime);
   }

   /**
    * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies the clearance of that collision
    */
   default double getFootStubClearance()
   {
      return get(SwingPlannerParameterKeys.footStubClearance);
   }

   /**
    * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies how much to shift if a collision is detected
    */
   default double getWaypointProportionShiftForStubAvoidance()
   {
      return get(SwingPlannerParameterKeys.waypointProportionShiftForStubAvoidance);
   }

   default boolean getDoInitialFastApproximation()
   {
      return get(SwingPlannerParameterKeys.doInitialFastApproximation);
   }

   /**
    * If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
    * along the swing foot trajectory and the planar regions in the environment.
    */
   default double getMinimumSwingFootClearance()
   {
      return get(SwingPlannerParameterKeys.minimumSwingFootClearance);
   }

   /**
    * If using the swing over planar regions module, this sets up how much less clearance is required on the fast approximation, since it doesn't usually have
    * the same amount of curve to the trajectory.
    */
   default double getFastApproximationLessClearance()
   {
      return get(SwingPlannerParameterKeys.fastApproximationLessClearance);
   }

   /**
    * If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
    */
   default int getNumberOfChecksPerSwing()
   {
      return get(SwingPlannerParameterKeys.numberOfChecksPerSwing);
   }

   /**
    * If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
    * collisions with the environment.
    */
   default int getMaximumNumberOfAdjustmentAttempts()
   {
      return get(SwingPlannerParameterKeys.maximumNumberOfAdjustmentAttempts);
   }

   /**
    * If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
    */
   default double getMaximumWaypointAdjustmentDistance()
   {
      return get(SwingPlannerParameterKeys.maximumWaypointAdjustmentDistance);
   }

   /**
    * If using the swing over planar regions module, this is the minimum distance that the swing waypoints will be adjusted by on each increment.
    */
   default double getMinimumAdjustmentIncrementDistance()
   {
      return get(SwingPlannerParameterKeys.minimumAdjustmentIncrementDistance);
   }

   /**
    * If using the swing over planar regions module, this is the maximum distance that the swing waypoints will be adjusted by on each increment.
    */
   default double getMaximumAdjustmentIncrementDistance()
   {
      return get(SwingPlannerParameterKeys.maximumAdjustmentIncrementDistance);
   }

   /**
    * If using the swing over planar regions module, this is the scale factor to be applied to the collision on each increment for adjustment.
    */
   default double getAdjustmentIncrementDistanceGain()
   {
      return get(SwingPlannerParameterKeys.adjustmentIncrementDistanceGain);
   }

   default double getMinimumHeightAboveFloorForCollision()
   {
      return get(SwingPlannerParameterKeys.minimumHeightAboveFloorForCollision);
   }

   default double getAdditionalSwingTimeIfExpanded()
   {
      return get(SwingPlannerParameterKeys.additionalSwingTimeIfExpanded);
   }

   default SwingPlannerParametersPacket getAsPacket()
   {
      SwingPlannerParametersPacket packet = new SwingPlannerParametersPacket();

      packet.setSwingHeightIfCollisionDetected(getSwingHeightIfCollisionDetected());
      packet.setMinimumSwingTime(getMinimumSwingTime());
      packet.setMaximumSwingTime(getMaximumSwingTime());
      packet.setFootStubClearance(getFootStubClearance());
      packet.setWaypointProportionShiftForStubAvoidance(getWaypointProportionShiftForStubAvoidance());

      packet.setDoInitialFastApproximation(getDoInitialFastApproximation());
      packet.setFastApproximationLessClearance(getFastApproximationLessClearance());
      packet.setMinimumSwingFootClearance(getMinimumSwingFootClearance());
      packet.setNumberOfChecksPerSwing(getNumberOfChecksPerSwing());
      packet.setMaximumNumberOfAdjustmentAttempts(getMaximumNumberOfAdjustmentAttempts());
      packet.setMaximumWaypointAdjustmentDistance(getMaximumWaypointAdjustmentDistance());
      packet.setMinimumAdjustmentIncrementDistance(getMinimumAdjustmentIncrementDistance());
      packet.setMaximumAdjustmentIncrementDistance(getMaximumAdjustmentIncrementDistance());
      packet.setAdjustmentIncrementDistanceGain(getAdjustmentIncrementDistanceGain());
      packet.setMinimumHeightAboveFloorForCollision(getMinimumHeightAboveFloorForCollision());
      packet.setAdditionalSwingTimeIfExpanded(getAdditionalSwingTimeIfExpanded());

      return packet;
   }
}
