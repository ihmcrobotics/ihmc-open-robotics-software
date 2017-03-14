package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandDesiredConfigurationBehavior extends AbstractBehavior
{
   private HandDesiredConfigurationMessage outgoingHandDesiredConfigurationMessage;
   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable hasPacketBeenSet;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime; // hardcoded, to be determined
   private final BooleanYoVariable trajectoryTimeElapsed;

   private final boolean DEBUG = false;

   public HandDesiredConfigurationBehavior(String name, CommunicationBridgeInterface outgoingCommunicationBridgeInterface, DoubleYoVariable yoTime)
   {
      super(name,outgoingCommunicationBridgeInterface);
      this.yoTime = yoTime;

      hasInputBeenSet = new BooleanYoVariable(getName() + "hasInputBeenSet", registry);
      hasPacketBeenSet = new BooleanYoVariable(getName() + "hasPacketBeenSet", registry);

      startTime = new DoubleYoVariable(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);

      trajectoryTimeElapsed = new BooleanYoVariable(getName() + "TrajectoryTimeElapsed", registry);
   }

   public void setInput(HandDesiredConfigurationMessage handDesiredConfigurationMessage)
   {
      this.outgoingHandDesiredConfigurationMessage = handDesiredConfigurationMessage;
      hasInputBeenSet.set(true);

      if (DEBUG)
         PrintTools.debug(this, "Input has been set: " + outgoingHandDesiredConfigurationMessage);
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSet.getBooleanValue() && outgoingHandDesiredConfigurationMessage != null)
         sendHandDesiredConfigurationToController();
   }

   private void sendHandDesiredConfigurationToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {

//         sendPacketToController(outgoingHandDesiredConfigurationMessage);
         
         outgoingHandDesiredConfigurationMessage.setDestination(PacketDestination.BROADCAST);
         
         sendPacket(outgoingHandDesiredConfigurationMessage);
         hasPacketBeenSet.set(true);
         startTime.set(yoTime.getDoubleValue());

         if (DEBUG)
            PrintTools.debug(this, "Sending HandDesiredConfigurationMessage to Controller: " + outgoingHandDesiredConfigurationMessage);
      }
   }


   @Override
   public void onBehaviorAborted()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         HandDesiredConfigurationMessage stopMessage = new HandDesiredConfigurationMessage(robotSide, HandConfiguration.STOP);
         stopMessage.setDestination(PacketDestination.UI);
         sendPacketToController(stopMessage);
         sendPacket(stopMessage);
      }
      isAborted.set(true);
   }

  

   @Override
   public void onBehaviorPaused()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         HandDesiredConfigurationMessage stopMessage = new HandDesiredConfigurationMessage(robotSide, HandConfiguration.STOP);
         stopMessage.setDestination(PacketDestination.UI);
         sendPacketToController(stopMessage);
         sendPacket(stopMessage);
      }
      isPaused.set(true);

      if (DEBUG)
         PrintTools.debug(this, "Pausing Behavior");
   }

   @Override
   public void onBehaviorResumed()
   {
      isPaused.set(false);
      hasPacketBeenSet.set(false);
      if (hasInputBeenSet())
      {
         sendHandDesiredConfigurationToController();
      }
      if (DEBUG)
         PrintTools.debug(this, "Resuming Behavior");
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue() && !isAborted.getBooleanValue();
   }

   @Override
   public void onBehaviorEntered()
   {
      if (hasInputBeenSet())
      {
         PrintTools.debug(this, "Re-Initializing Behavior");
      }
      hasInputBeenSet.set(false);
      hasPacketBeenSet.set(false);

      isPaused.set(false);
      isAborted.set(false);
      trajectoryTime.set(1.0); //TODO hardCoded to be determined

      trajectoryTimeElapsed.set(false);
   }

   @Override
   public void onBehaviorExited()
   {
      hasInputBeenSet.set(false);
      hasPacketBeenSet.set(false);
      outgoingHandDesiredConfigurationMessage = null;
      isPaused.set(false);
      isAborted.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(false);
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
