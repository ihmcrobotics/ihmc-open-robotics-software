package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager.GlobalStatusMessageListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class ControllerNetworkSubscriber
{
   private final ControllerCommandInputManager controllerCommandInputManager;
   private final ControllerStatusOutputManager controllerStatusOutputManager;
   private final HumanoidGlobalDataProducer globalDataProducer;

   public ControllerNetworkSubscriber(ControllerCommandInputManager controllerCommandInputManager, ControllerStatusOutputManager controllerStatusOutputManager,
         HumanoidGlobalDataProducer globalDataProducer)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.globalDataProducer = globalDataProducer;

      createAllSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
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
               globalDataProducer.notifyInvalidPacketReceived(messageClass, errorMessage);
               return;
            }

            controllerCommandInputManager.submitMessage(message);
         }
      };
      globalDataProducer.attachListener(messageClass, packetConsumer);
   }

   private void createGlobalStatusMessageListener()
   {
      GlobalStatusMessageListener globalStatusMessageListener = new GlobalStatusMessageListener()
      {
         @Override
         public void receivedNewMessageStatus(StatusPacket<?> statusMessage)
         {
            globalDataProducer.queueDataToSend(statusMessage);
         }
      };
      controllerStatusOutputManager.attachGlobalStatusMessageListener(globalStatusMessageListener);
   }
}
