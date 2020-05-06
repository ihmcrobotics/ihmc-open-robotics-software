package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisOrientationTrajectoryBehavior extends AbstractBehavior
{
   private PelvisOrientationTrajectoryMessage outgoingPelvisOrientationTrajectoryMessage;

   private final YoBoolean hasPacketBeenSent;
   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final YoBoolean trajectoryTimeElapsed;

   private final IHMCROS2Publisher<PelvisOrientationTrajectoryMessage> publisher;

   public PelvisOrientationTrajectoryBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, ros2Node);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new YoBoolean(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);

      publisher = createPublisherForController(PelvisOrientationTrajectoryMessage.class);
   }

   public void setInput(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      this.outgoingPelvisOrientationTrajectoryMessage = pelvisOrientationTrajectoryMessage;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingPelvisOrientationTrajectoryMessage != null))
      {
         sendPelvisPosePacketToController();
      }
   }

   private void sendPelvisPosePacketToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         publisher.publish(outgoingPelvisOrientationTrajectoryMessage);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingPelvisOrientationTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      hasPacketBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
      hasBeenInitialized.set(true);
   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override //TODO: Not currently implemented for this behavior
   public void onBehaviorResumed()
   {
      isPaused.set(false);
      startTime.set(yoTime.getDoubleValue());
   }

   @Override
   public boolean isDone()
   {
      if (hasPacketBeenSent.getBooleanValue())
      {
         if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
            trajectoryTimeElapsed.set(false);
         else
            trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

         return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
      }
      else
         return false;
   }

   public boolean hasInputBeenSet()
   {
      return outgoingPelvisOrientationTrajectoryMessage != null;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }
}
