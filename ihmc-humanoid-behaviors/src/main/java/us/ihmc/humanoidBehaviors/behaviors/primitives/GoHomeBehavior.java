package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class GoHomeBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   protected HumanoidBodyPart bodyPart;

   protected GoHomeMessage outgoingMessage;

   protected final YoBoolean hasPacketBeenSent;
   protected final YoDouble yoTime;
   protected final YoDouble startTime;
   protected final YoDouble trajectoryTime;
   private final YoDouble trajectoryTimeElapsed;

   protected final YoBoolean hasInputBeenSet;
   private final YoBoolean isDone;

   private final IHMCROS2Publisher<GoHomeMessage> goHomePublisher;
   private final IHMCROS2Publisher<StopAllTrajectoryMessage> stopAllTrajectoryPublisher;

   public GoHomeBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, null, ros2Node, yoTime);
   }

   public GoHomeBehavior(String robotName, String namePrefix, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, namePrefix, ros2Node);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);

      hasInputBeenSet = new YoBoolean(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      isDone = new YoBoolean(behaviorNameFirstLowerCase + "IsDone", registry);

      goHomePublisher = createPublisherForController(GoHomeMessage.class);
      stopAllTrajectoryPublisher = createPublisherForController(StopAllTrajectoryMessage.class);
   }

   public void setInput(GoHomeMessage goHomeMessage)
   {
      outgoingMessage = goHomeMessage;

      bodyPart = HumanoidBodyPart.fromByte(goHomeMessage.getHumanoidBodyPart());
      startTime.set(yoTime.getDoubleValue());
      trajectoryTime.set(goHomeMessage.getTrajectoryTime());

      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());

      if (!isDone.getBooleanValue() && !isPaused.getBooleanValue() && !isAborted.getBooleanValue()
            && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime.getDoubleValue())
      {
         if (DEBUG)
            PrintTools.debug(this, bodyPart + " GoHomeBehavior setting isDone = true");
         isDone.set(true);
      }

      if (!hasPacketBeenSent.getBooleanValue() && (outgoingMessage != null))
      {
         sendOutgoingPacketToControllerAndNetworkProcessor();
      }
   }

   private void sendOutgoingPacketToControllerAndNetworkProcessor()
   {

      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         goHomePublisher.publish(outgoingMessage);

         hasPacketBeenSent.set(true);

         if (DEBUG)
            PrintTools.debug(this, "sending packet to controller and network processor: " + outgoingMessage);
      }
   }

   private void stopArmMotion()
   {
      if (outgoingMessage != null)
      {
         stopAllTrajectoryPublisher.publish(new StopAllTrajectoryMessage());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      isPaused.set(false);
      isDone.set(false);
      hasBeenInitialized.set(true);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
   }

   @Override
   public void onBehaviorExited()
   {
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      hasInputBeenSet.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
      isDone.set(false);
   }

   @Override
   public void onBehaviorAborted()
   {
      stopArmMotion();
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {
      if (isPaused.getBooleanValue())
      {
         return;
      }
      else
      {
         stopArmMotion();
         isPaused.set(true);
      }
   }

   @Override
   public void onBehaviorResumed()
   {
      if (!isPaused.getBooleanValue())
      {
         return;
      }
      else
      {
         isPaused.set(false);

         if (hasInputBeenSet())
         {
            sendOutgoingPacketToControllerAndNetworkProcessor();
         }
      }
   }

   @Override
   public boolean isDone(double timeinState)
   {
      return isDone.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
