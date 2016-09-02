package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class HeadTrajectoryBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent;
   private HeadTrajectoryMessage outgoingHeadTrajectoryMessage;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;

   public HeadTrajectoryBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(null, outgoingCommunicationBridge, yoTime);
   }
   
   public HeadTrajectoryBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge);
      
      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      packetHasBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
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
      if (!isPaused.getBooleanValue() &&!isStopped.getBooleanValue())
      {
         outgoingHeadTrajectoryMessage.setDestination(PacketDestination.UI);
         sendPacketToNetworkProcessor(outgoingHeadTrajectoryMessage);
         sendPacketToController(outgoingHeadTrajectoryMessage);
         
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingHeadTrajectoryMessage.getTrajectoryTime());
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
      outgoingHeadTrajectoryMessage = null;

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
	   if (outgoingHeadTrajectoryMessage != null)
		   return true;
	   else
		   return false;
   }
}
