package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeadTrajectoryBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent;
   private HeadTrajectoryMessage outgoingHeadTrajectoryMessage;

   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;

   public HeadTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }
   
   public HeadTrajectoryBehavior(String namePrefix, CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);
      
      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      packetHasBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
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
      if (!isPaused.getBooleanValue() &&!isAborted.getBooleanValue())
      {
         outgoingHeadTrajectoryMessage.setDestination(PacketDestination.UI);
         sendPacket(outgoingHeadTrajectoryMessage);
         sendPacketToController(outgoingHeadTrajectoryMessage);
         
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHeadTrajectoryMessage.getTrajectoryTime());
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
   public boolean isDone()
   {
      boolean trajectoryTimeElapsed = yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue();

      return trajectoryTimeElapsed && !isPaused.getBooleanValue();
   }

   

   
   public boolean hasInputBeenSet() {
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
