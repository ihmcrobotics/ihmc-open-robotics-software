package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeadTrajectoryBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent;
   private HeadTrajectoryMessage outgoingHeadTrajectoryMessage;

   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final IHMCROS2Publisher<HeadTrajectoryMessage> publisher;

   public HeadTrajectoryBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, null, ros2Node, yoTime);
   }

   public HeadTrajectoryBehavior(String robotName, String namePrefix, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, namePrefix, ros2Node);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      packetHasBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);

      publisher = createPublisherForController(HeadTrajectoryMessage.class);
   }

   public void setInput(HeadTrajectoryMessage headTrajectoryMessage)
   {
      this.outgoingHeadTrajectoryMessage = headTrajectoryMessage;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingHeadTrajectoryMessage != null))
      {
         sendHeadOrientationPacketToController();
      }
   }

   private void sendHeadOrientationPacketToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         publisher.publish(outgoingHeadTrajectoryMessage);

         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHeadTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);

      hasBeenInitialized.set(true);
   }

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      outgoingHeadTrajectoryMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      trajectoryTime.set(Double.NaN);
   }

   @Override
   public boolean isDone(double timeinState)
   {
      boolean trajectoryTimeElapsed = yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue();

      return trajectoryTimeElapsed && !isPaused.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      if (outgoingHeadTrajectoryMessage != null)
         return true;
      else
         return false;
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
