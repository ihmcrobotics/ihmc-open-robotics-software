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
      defaultInitialize();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public boolean isDone()
   {
      return defaultIsDone();
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void resume()
   {
      defaultResume();
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
      return true;
   }
}
