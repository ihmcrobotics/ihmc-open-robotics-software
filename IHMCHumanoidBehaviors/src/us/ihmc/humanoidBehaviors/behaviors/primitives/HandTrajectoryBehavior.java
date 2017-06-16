package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTrajectoryBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   protected RobotSide robotSide;

   protected HandTrajectoryMessage outgoingMessage;

   protected final YoBoolean hasPacketBeenSent;
   protected final YoDouble yoTime;
   protected final YoDouble startTime;
   protected final YoDouble trajectoryTime;
   private final YoDouble trajectoryTimeElapsed;

   protected final YoBoolean hasInputBeenSet;
   protected final YoBoolean hasStatusBeenReceived;
   private final YoBoolean isDone;

   public HandTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public HandTrajectoryBehavior(String namePrefix, CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

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
      hasStatusBeenReceived = new YoBoolean(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new YoBoolean(behaviorNameFirstLowerCase + "IsDone", registry);
   }

   public void setInput(HandTrajectoryMessage armTrajectoryMessage)
   {
      outgoingMessage = armTrajectoryMessage;

      robotSide = armTrajectoryMessage.getRobotSide();
      startTime.set(yoTime.getDoubleValue());
      trajectoryTime.set(armTrajectoryMessage.getTrajectoryTime());

      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());

      if (!isDone.getBooleanValue() && hasInputBeenSet() && !isPaused.getBooleanValue() && !isAborted.getBooleanValue()
            && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime.getDoubleValue())
      {
         if (DEBUG)
            PrintTools.debug(this, robotSide + " HandTrajectoryBehavior setting isDone = true");
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
        // outgoingMessage.setDestination(PacketDestination.UI);

         //sendPacketToController(outgoingMessage);
         sendPacket(outgoingMessage);

         hasPacketBeenSent.set(true);

         if (DEBUG)
            PrintTools.debug(this, "sending packet to controller and network processor: " + outgoingMessage);
      }
   }

   private void stopArmMotion()
   {
      if (outgoingMessage != null)
      {
         StopAllTrajectoryMessage pausePacket = new StopAllTrajectoryMessage();
         pausePacket.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(pausePacket);
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      hasStatusBeenReceived.set(false);
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
      hasStatusBeenReceived.set(false);

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
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
