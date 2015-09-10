package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ComHeightBehavior extends BehaviorInterface
{
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private final BooleanYoVariable trajectoryTimeElapsed = new BooleanYoVariable(getName() + "TrajectoryTimeElapsed", registry);
   private ComHeightPacket outgoingComHeightPacket;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;

   public ComHeightBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      startTime = new DoubleYoVariable(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      this.yoTime = yoTime;
   }

   public void setInput(ComHeightPacket comHeightPacket)
   {
      this.outgoingComHeightPacket = comHeightPacket;
      hasInputBeenSet.set(true);
   }

   public void kneelDown()
   {
      ComHeightPacket packet = new ComHeightPacket(ComHeightPacket.MIN_COM_HEIGHT + 0.08);  // ComHeightPacket.MIN_COM_HEIGHT + 0.15
      setInput(packet);
   }

   public void goToHomeHeight()
   {
      ComHeightPacket packet = new ComHeightPacket(0.0);
      setInput(packet);
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() &&  hasInputBeenSet())
      {
         sendComHeightToController();
      }
   }

   private void sendComHeightToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {      
         outgoingComHeightPacket.setDestination(PacketDestination.UI);  
         sendPacketToController(outgoingComHeightPacket);
         sendPacketToNetworkProcessor(outgoingComHeightPacket);
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingComHeightPacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isStopped.set(false);
      
      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);
      
      hasBeenInitialized.set(true);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);
      outgoingComHeightPacket = null;

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
      
      isPaused.set(false);
      isStopped.set(false);
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
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
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

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
