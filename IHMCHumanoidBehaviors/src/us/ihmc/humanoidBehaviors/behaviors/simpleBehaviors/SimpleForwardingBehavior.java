package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;

public class SimpleForwardingBehavior extends BehaviorInterface
{
   private final BehaviorCommunicationBridge communicationBridge;

   public SimpleForwardingBehavior(BehaviorCommunicationBridge communicationBridge)
   {
      super(communicationBridge);

      this.communicationBridge = communicationBridge;
   }

   @Override
   public void doControl()
   {
      if (communicationBridge != null)
         communicationBridge.setPacketPassThrough(true);
   }

   @Override
   public void initialize()
   {
      if (communicationBridge != null)
         communicationBridge.setPacketPassThrough(true);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      if (communicationBridge != null)
         communicationBridge.setPacketPassThrough(false);
   }

   @Override
   public void stop()
   {

   }

   @Override
   public void pause()
   {

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void resume()
   {
      if (communicationBridge != null)
         communicationBridge.setPacketPassThrough(true);
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
		   return false;
   }
}
