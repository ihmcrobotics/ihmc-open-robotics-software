package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;

public class SimpleForwardingBehavior extends AbstractBehavior
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
   public boolean isDone()
   {
      return false;
   }

   

   @Override
   public void resume()
   {
      if (communicationBridge != null)
         communicationBridge.setPacketPassThrough(true);
   }

  
   
   public boolean hasInputBeenSet() {
		   return false;
   }
}
