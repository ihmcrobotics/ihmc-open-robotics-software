package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class WholeBodyTrajectoryBehavior extends AbstractBehavior
{
   protected WholeBodyTrajectoryMessage outgoingMessage;
   
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private final BooleanYoVariable trajectoryTimeElapsed = new BooleanYoVariable(getName() + "TrajectoryTimeElapsed", registry);
   
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   
   public WholeBodyTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }

   public WholeBodyTrajectoryBehavior(String namePrefix, CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);

      startTime = new DoubleYoVariable(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(getName() + "TrajectoryTime", registry);
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
