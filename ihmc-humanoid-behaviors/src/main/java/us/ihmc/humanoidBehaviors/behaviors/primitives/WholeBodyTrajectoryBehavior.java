package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotSide;

public class WholeBodyTrajectoryBehavior extends AbstractBehavior
{
   protected WholeBodyTrajectoryMessage outgoingMessage;
   
   private final YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet" + behaviorName, registry);
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private final YoBoolean trajectoryTimeElapsed = new YoBoolean(getName() + "TrajectoryTimeElapsed", registry);
   
   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   
   public WholeBodyTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public WholeBodyTrajectoryBehavior(String namePrefix, CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

      startTime = new YoDouble(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      this.yoTime = yoTime;
   }
   
   public void setInput(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      outgoingMessage = wholebodyTrajectoryMessage;
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() &&  hasInputBeenSet())
      {
         sendMessageToController();
      }
   }

   private void sendMessageToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {      
         outgoingMessage.setDestination(PacketDestination.CONTROLLER);  
         sendPacketToController(outgoingMessage);
         sendPacket(outgoingMessage);
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         
         double getTrajectoryTime = 0;
         if(outgoingMessage.getHandTrajectoryMessage(RobotSide.RIGHT) != null)
            getTrajectoryTime = outgoingMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryTime();
         if(outgoingMessage.getHandTrajectoryMessage(RobotSide.LEFT) != null)
            getTrajectoryTime = outgoingMessage.getHandTrajectoryMessage(RobotSide.LEFT).getTrajectoryTime();
         
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
}
