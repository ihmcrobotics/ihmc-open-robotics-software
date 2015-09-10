package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.FormattingTools;

public class HeadOrientationBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent;
   private HeadOrientationPacket outgoingHeadOrientationPacket;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;

   public HeadOrientationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }
   
   public HeadOrientationBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);
      
      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      packetHasBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
   }

   public void setInput(HeadOrientationPacket headOrientationPacket)
   {
      this.outgoingHeadOrientationPacket = headOrientationPacket;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingHeadOrientationPacket != null))
      {
         sendHeadOrientationPacketToController();
      }
   }

   private void sendHeadOrientationPacketToController()
   {
      if (!isPaused.getBooleanValue() &&!isStopped.getBooleanValue())
      {
         outgoingHeadOrientationPacket.setDestination(PacketDestination.UI);
         sendPacketToNetworkProcessor(outgoingHeadOrientationPacket);
         sendPacketToController(outgoingHeadOrientationPacket);
         
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHeadOrientationPacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);
      
      isPaused.set(false);
      isStopped.set(false);
      
      hasBeenInitialized.set(true);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      packetHasBeenSent.set(false);
      outgoingHeadOrientationPacket = null;

      isPaused.set(false);
      isStopped.set(false);
      
      trajectoryTime.set(Double.NaN);      
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      boolean trajectoryTimeElapsed = yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue();

      return trajectoryTimeElapsed && !isPaused.getBooleanValue();
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }
   
   public boolean hasInputBeenSet() {
	   if (outgoingHeadOrientationPacket != null)
		   return true;
	   else
		   return false;
   }
}
