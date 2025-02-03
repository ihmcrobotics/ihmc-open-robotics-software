package us.ihmc.perception;

import controller_msgs.msg.dds.PlanOffsetStatus;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * This class is meant to keep track of the drift in Z that is being calculated in the controller on the robot.
 * This class takes in a {@link ControllerFootstepQueueMonitor} in order to subscribe to the {@link PlanOffsetStatus}.
 * It then keeps track of the current drift since walking has started.
 */
public class RapidHeightMapDriftOffset
{
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final Vector3D mostRecentPlanOffsetProcessed = new Vector3D();
   private final Vector3D incrementalOffset = new Vector3D();

   public RapidHeightMapDriftOffset(ControllerFootstepQueueMonitor controllerFootstepQueueMonitor)
   {
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
   }

   public void reset()
   {
      mostRecentPlanOffsetProcessed.setToZero();
   }

   /**
    * The method returns the amount of drift that has happened in meters from the controller.
    * If no drift has happened then {@link Float#NaN} is returned
    */
   public float getUpdateDriftOffset()
   {
      incrementalOffset.setToNaN();

      if (controllerFootstepQueueMonitor.isWalkingStarted())
      {
         // We reset this because the controller resets the drift on its end. So we need to reset ours as well.
         // The existing drift is already captured in the height map by the previous offsets
         mostRecentPlanOffsetProcessed.setToZero();
      }

      // While the robot isn't walking, this message will be null
      PlanOffsetStatus latestPlanOffsetMessage = controllerFootstepQueueMonitor.getPlanOffsetMessage();
      if (latestPlanOffsetMessage == null)
         return Float.NaN;

      Vector3D latestPlanOffset = latestPlanOffsetMessage.getOffsetVector();

      if (controllerFootstepQueueMonitor.isFootstepStarted() && latestPlanOffset != null)
      {
         incrementalOffset.set(latestPlanOffset);
         // We subtract the drift that has already been accounted for to only adjust by the new drift
         incrementalOffset.sub(mostRecentPlanOffsetProcessed);
         // Our internal tracker needs to add the drift because that is the new value being accounted for, so add it to the running total
         mostRecentPlanOffsetProcessed.add(incrementalOffset);
      }

      return (float) incrementalOffset.getZ();
   }
}
