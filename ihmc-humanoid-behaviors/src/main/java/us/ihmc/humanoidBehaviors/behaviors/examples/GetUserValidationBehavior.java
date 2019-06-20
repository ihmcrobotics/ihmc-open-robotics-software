package us.ihmc.humanoidBehaviors.behaviors.examples;

import controller_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.ros2.Ros2Node;

public class GetUserValidationBehavior extends AbstractBehavior implements CoactiveDataListenerInterface
{

   private boolean validated = false;

   CommunicationBridge coactiveBehaviorsNetworkManager;
   private boolean recievedMessage = false;

   public GetUserValidationBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, ros2Node);
      //      coactiveBehaviorsNetworkManager = ros2Node; // FIXME I broke it when switching to pub-sub (Sylvain)
      //      coactiveBehaviorsNetworkManager.addListeners(this);
   }

   @Override
   public void doControl()
   {

   }

   public boolean isValidated()
   {
      return validated;
   }

   @Override
   public boolean isDone()
   {
      return recievedMessage;
   }

   @Override
   public void onBehaviorEntered()
   {
      //reset necessary values so this behavior can run again properly
      publishTextToSpeech("Waiting For User Validation");
      validated = false;
      recievedMessage = false;
      //maybe let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 1);
      coactiveBehaviorsNetworkManager.sendToUI("WaitingForValidation", 1);
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Got User Validation");
      //let the UI know this specific behavior has ended
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 0);
      coactiveBehaviorsNetworkManager.sendToUI("WaitingForValidation", 0);

   }

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {
      if (data.getKeyAsString().equalsIgnoreCase("validate"))
      {
         if (data.getValue() == 1)
         {
            validated = true;
         }
         else
         {
            validated = false;

         }
         recievedMessage = true;
      }
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
