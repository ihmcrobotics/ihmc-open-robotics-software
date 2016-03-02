package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.List;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class ControllerNetworkSubscriber
{
   private final ControllerCommandInputManager controllerCommandInputManager;
   private final HumanoidGlobalDataProducer globalDataProducer;

   public ControllerNetworkSubscriber(ControllerCommandInputManager controllerCommandInputManager, HumanoidGlobalDataProducer globalDataProducer)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.globalDataProducer = globalDataProducer;
      
      createAllSubscribersForSupportedMessages();
   }

   private <T extends Packet<T>> void createAllSubscribersForSupportedMessages()
   {

      List<Class<? extends Packet<?>>> listOfSupportedMessages = controllerCommandInputManager.getListOfSupportedMessages();
      for (int i = 0; i < listOfSupportedMessages.size(); i++)
      {
         @SuppressWarnings("unchecked")
         Class<T> messageClass = (Class<T>) listOfSupportedMessages.get(i);
         createSubscriber(messageClass);
      }
   }

   private <T extends Packet<T>> void createSubscriber(final Class<T> messageClass)
   {
      PacketConsumer<T> packetConsumer = new PacketConsumer<T>()
      {

         @Override
         public void receivedPacket(T message)
         {
            String errorMessage = message.validateMessage();
            if (errorMessage != null)
            {
               if (globalDataProducer != null)
                  globalDataProducer.notifyInvalidPacketReceived(messageClass, errorMessage);
               return;
            }

            controllerCommandInputManager.submitMessage(message);
         }
      };
      globalDataProducer.attachListener(messageClass, packetConsumer);
   }
}
