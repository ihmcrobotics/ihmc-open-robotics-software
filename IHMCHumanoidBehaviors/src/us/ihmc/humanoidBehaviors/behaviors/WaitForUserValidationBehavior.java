package us.ihmc.humanoidBehaviors.behaviors;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class WaitForUserValidationBehavior extends AbstractBehavior
{

   private BooleanYoVariable validClicked;
   private BooleanYoVariable validAcknoledged;

   ExecutorService executorService = Executors.newFixedThreadPool(2);

   public WaitForUserValidationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, BooleanYoVariable validClicked,
         BooleanYoVariable validAcknoledged)
   {
      super(outgoingCommunicationBridge);
      this.validAcknoledged = validAcknoledged;
      this.validClicked = validClicked;

   }

   public void reset()
   {
      validAcknoledged.set(false);
   }

   @Override
   public void doControl()
   {
      if (validClicked.getBooleanValue())
      {
         validAcknoledged.set(true);
      }
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
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   public boolean isDone()
   {
      return true;
      
//      return validAcknoledged.getBooleanValue();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      // TODO Auto-generated method stub
      return true;
   }

   @Override
   public void initialize()
   {
      defaultPostBehaviorCleanup();
   }
}
