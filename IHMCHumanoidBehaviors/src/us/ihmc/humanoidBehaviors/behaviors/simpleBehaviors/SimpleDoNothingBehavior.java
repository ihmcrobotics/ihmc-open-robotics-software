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
      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void finalize()
   {
      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
      isPaused.set(false);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
      isStopped.set(false);
   }

   @Override
   public boolean isDone()
   {
      boolean ret = !isPaused.getBooleanValue() && !isStopped.getBooleanValue();
      return ret;
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void resume()
   {
      isPaused.set(false);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {

   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {

   }

   public boolean hasInputBeenSet()
   {
      return false;
   }
}
