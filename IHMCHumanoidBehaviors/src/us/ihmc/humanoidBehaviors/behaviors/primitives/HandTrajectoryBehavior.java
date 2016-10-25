package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class HandTrajectoryBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   protected RobotSide robotSide;

   protected HandTrajectoryMessage outgoingMessage;

   protected final BooleanYoVariable hasPacketBeenSent;
   protected final DoubleYoVariable yoTime;
   protected final DoubleYoVariable startTime;
   protected final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeElapsed;

   protected final BooleanYoVariable hasInputBeenSet;
   protected final BooleanYoVariable hasStatusBeenReceived;
   private final BooleanYoVariable isDone;

   public HandTrajectoryBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public HandTrajectoryBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);

      hasInputBeenSet = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      hasStatusBeenReceived = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new BooleanYoVariable(behaviorNameFirstLowerCase + "IsDone", registry);
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
         outgoingMessage.setDestination(PacketDestination.UI);

         sendPacketToController(outgoingMessage);
         sendPacketToNetworkProcessor(outgoingMessage);

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
   public void initialize()
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
   public void doPostBehaviorCleanup()
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
   public void abort()
   {
      stopArmMotion();
      isAborted.set(true);
   }

   @Override
   public void pause()
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
   public void resume()
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
