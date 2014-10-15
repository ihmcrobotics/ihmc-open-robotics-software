package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;

public class SimpleDoNothingBehavior extends BehaviorInterface
{
   public SimpleDoNothingBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   @Override
   public void doControl()
   {

   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void finalize()
   {
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
