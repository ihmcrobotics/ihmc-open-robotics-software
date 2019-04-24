package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyTrajectoryBehavior extends AbstractBehavior
{
   protected WholeBodyTrajectoryMessage outgoingMessage;

   private final YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet" + behaviorName, registry);
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private final YoBoolean trajectoryTimeElapsed = new YoBoolean(getName() + "TrajectoryTimeElapsed", registry);

   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;

   private final IHMCROS2Publisher<WholeBodyTrajectoryMessage> publisher;

   public WholeBodyTrajectoryBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, null, ros2Node, yoTime);
   }

   public WholeBodyTrajectoryBehavior(String robotName, String namePrefix, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, namePrefix, ros2Node);

      startTime = new YoDouble(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      this.yoTime = yoTime;

      publisher = createPublisherForController(WholeBodyTrajectoryMessage.class);
   }

   public void setInput(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      outgoingMessage = wholebodyTrajectoryMessage;
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
         outgoingMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
         publisher.publish(outgoingMessage);
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());

         double getTrajectoryTime = 0;
         if (outgoingMessage.getRightHandTrajectoryMessage() != null)
            getTrajectoryTime = outgoingMessage.getRightHandTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime();
         if (outgoingMessage.getLeftHandTrajectoryMessage() != null)
            getTrajectoryTime = outgoingMessage.getLeftHandTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime();

         trajectoryTime.set(getTrajectoryTime);
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

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);
      outgoingMessage = null;

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);

      isPaused.set(false);
      isAborted.set(false);
      PrintTools.info("WholeBodyTrajectoryBehavior Exited");
   }

   @Override
   public boolean isDone(double timeinState)
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
}
