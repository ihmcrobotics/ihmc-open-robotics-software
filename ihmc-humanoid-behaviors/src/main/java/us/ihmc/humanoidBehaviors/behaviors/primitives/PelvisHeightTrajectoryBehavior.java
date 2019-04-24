package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisHeightTrajectoryBehavior extends AbstractBehavior
{
   private final YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet" + behaviorName, registry);
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private final YoBoolean trajectoryTimeElapsed = new YoBoolean(getName() + "TrajectoryTimeElapsed", registry);
   private PelvisHeightTrajectoryMessage outgoingPelvisHeightTrajectoryMessage;

   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final IHMCROS2Publisher<PelvisHeightTrajectoryMessage> publisher;

   public PelvisHeightTrajectoryBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, ros2Node);
      startTime = new YoDouble(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      this.yoTime = yoTime;

      publisher = createPublisherForController(PelvisHeightTrajectoryMessage.class);
   }

   public void setInput(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      this.outgoingPelvisHeightTrajectoryMessage = pelvisHeightTrajectoryMessage;
      System.out.println("Pelvis height "
            + pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().getLast().getPosition().getZ());
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && hasInputBeenSet())
      {
         sendMessageToController();
      }
   }

   private void sendMessageToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         publisher.publish(outgoingPelvisHeightTrajectoryMessage);
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingPelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().getLast().getTime());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);

      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);

      hasBeenInitialized.set(true);
   }

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);
      outgoingPelvisHeightTrajectoryMessage = null;

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }
}
