package us.ihmc.humanoidBehaviors.behaviors.examples;

import controller_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;

public class GetUserValidationBehavior extends AbstractBehavior implements CoactiveDataListenerInterface
{

   private boolean validated = false;

   CommunicationBridge coactiveBehaviorsNetworkManager;
   private boolean recievedMessage = false;

   public GetUserValidationBehavior(CommunicationBridge communicationBridge)
   {
      super(communicationBridge);
      coactiveBehaviorsNetworkManager = communicationBridge;
      coactiveBehaviorsNetworkManager.addListeners(this);
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
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Waiting For User Validation");
      sendPacket(p1);

      validated = false;
      recievedMessage = false;
      //maybe let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 1);
      coactiveBehaviorsNetworkManager.sendToUI("WaitingForValidation", 1);
   }

   @Override
   public void onBehaviorExited()
   {
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Got User Validation");
      sendPacket(p1);
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
