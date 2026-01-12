package us.ihmc.perception.gpuMapping;

import controller_msgs.msg.dds.PlanOffsetStatus;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;

/**
 * This class is meant to keep track of the drift in Z that is being calculated in the controller on the robot.
 * This class takes in a {@link ControllerFootstepQueueMonitor} in order to subscribe to the {@link PlanOffsetStatus}.
 * It then keeps track of the current drift since walking has started.
 */
public class HeightMapDriftOffset
{
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final Vector3D previousPlanOffsetProcessed = new Vector3D();
   private final Vector3D incrementalOffset = new Vector3D();

   public HeightMapDriftOffset(ControllerFootstepQueueMonitor controllerFootstepQueueMonitor)
   {
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
   }

   public void reset()
   {
      previousPlanOffsetProcessed.setToZero();
   }

   /**
    * The method returns the amount of drift that has happened in meters from the controller.
    * If no drift has happened then {@link Float#NaN} is returned
    */
   public float getUpdateDriftOffset()
   {
      incrementalOffset.setToZero();

      if (controllerFootstepQueueMonitor.pollIsWalking() || controllerFootstepQueueMonitor.getReceivedNewFootstepPlanWithOverride())
      {
         // We reset this because the controller resets the drift on its end. So we need to reset ours as well.
         // The existing drift is already captured in the height map by the previous offsets
         previousPlanOffsetProcessed.setToZero();
      }

      // While the robot isn't walking, this message will be null
      PlanOffsetStatus latestPlanOffsetMessage = controllerFootstepQueueMonitor.pollPlanOffsetMessage();
      if (latestPlanOffsetMessage == null)
         return 0.0f;

      Vector3D latestPlanOffset = latestPlanOffsetMessage.getOffsetVector();

      if (controllerFootstepQueueMonitor.isFootstepStarted() && latestPlanOffset != null)
      {
         incrementalOffset.set(latestPlanOffset);
         // We subtract the drift that has already been accounted for to only adjust by the new drift
         incrementalOffset.sub(previousPlanOffsetProcessed);
         previousPlanOffsetProcessed.set(latestPlanOffset);
      }

      return (float) incrementalOffset.getZ();
   }
}
