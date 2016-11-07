package us.ihmc.humanoidBehaviors.behaviors.examples;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;

public class UserValidationExampleBehavior extends AbstractBehavior implements CoactiveDataListenerInterface
{

   private boolean validated = false;

   CommunicationBridge coactiveBehaviorsNetworkManager;

   public UserValidationExampleBehavior(CommunicationBridge communicationBridge)
   {
      super(communicationBridge);
      coactiveBehaviorsNetworkManager =communicationBridge;
      coactiveBehaviorsNetworkManager.addListeners(this);
   }

   @Override
   public void doControl()
   {

   }

   @Override
   public boolean isDone()
   {
      return validated;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      //reset necessary values so this behavior can run again properly
      validated = false;
      //let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 1);
      coactiveBehaviorsNetworkManager.sendToUI("WaitingForValidation", 1);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      //let the UI know this specific behavior has ended
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 0);
      coactiveBehaviorsNetworkManager.sendToUI("WaitingForValidation", 0);

   }

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {
      if (data.key.equalsIgnoreCase("validate"))
      {
         if (data.value == 1)
         {
            validated = true;
         }
      }
   }
}
