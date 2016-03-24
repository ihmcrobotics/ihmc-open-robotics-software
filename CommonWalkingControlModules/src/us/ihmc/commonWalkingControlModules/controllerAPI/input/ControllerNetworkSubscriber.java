package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.GlobalStatusMessageListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;

public class ControllerNetworkSubscriber implements Runnable, CloseableAndDisposable
{
   private final int buffersCapacity = 16;
   private final CommandInputManager controllerCommandInputManager;
   private final StatusMessageOutputManager controllerStatusOutputManager;
   private final PacketCommunicator packetCommunicator;
   private final PeriodicThreadScheduler scheduler;

   private final List<Class<? extends StatusPacket<?>>> listOfSupportedStatusMessages;
   private final Map<Class<? extends StatusPacket<?>>, ConcurrentRingBuffer<? extends StatusPacket<?>>> statusMessageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends StatusPacket<?>>, Builder<? extends StatusPacket<?>>> statusMessageClassToBuilderMap = new HashMap<>();

   public ControllerNetworkSubscriber(CommandInputManager controllerCommandInputManager, StatusMessageOutputManager controllerStatusOutputManager,
         CloseableAndDisposableRegistry closeAndDisposeRegistry, PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.scheduler = scheduler;
      this.packetCommunicator = packetCommunicator;
      listOfSupportedStatusMessages = controllerStatusOutputManager.getListOfSupportedMessages();
      listOfSupportedStatusMessages.add(InvalidPacketNotificationPacket.class);

      createAllSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
      createAllStatusMessageBuffers();

      scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
      closeAndDisposeRegistry.registerCloseableAndDisposable(this);
   }

   @SuppressWarnings("unchecked")
   private <T extends StatusPacket<T>> void createAllStatusMessageBuffers()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         Class<T> statusMessageClass = (Class<T>) listOfSupportedStatusMessages.get(i);
         Builder<T> builder = CommandInputManager.createBuilderWithEmptyConstructor(statusMessageClass);
         ConcurrentRingBuffer<T> newBuffer = new ConcurrentRingBuffer<>(builder, buffersCapacity);
         statusMessageClassToBufferMap.put(statusMessageClass, newBuffer);
         statusMessageClassToBuilderMap.put(statusMessageClass, CommandInputManager.createBuilderWithEmptyConstructor(statusMessageClass));
      }
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
               reportInvalidMessage(messageClass, errorMessage);
               return;
            }

            controllerCommandInputManager.submitMessage(message);
         }

         private void reportInvalidMessage(Class<T> messageClass, String errorMessage)
         {
            ConcurrentRingBuffer<?> buffer = statusMessageClassToBufferMap.get(InvalidPacketNotificationPacket.class);

            InvalidPacketNotificationPacket next = (InvalidPacketNotificationPacket) buffer.next();

            if (next != null)
            {
               next.set(messageClass, errorMessage);
               buffer.commit();
            }
         }
      };
      packetCommunicator.attachListener(messageClass, packetConsumer);
   }

   private void createGlobalStatusMessageListener()
   {
      GlobalStatusMessageListener globalStatusMessageListener = new GlobalStatusMessageListener()
      {
         @Override
         public void receivedNewMessageStatus(StatusPacket<?> statusMessage)
         {
            copyData(statusMessage);
         }

         @SuppressWarnings("unchecked")
         private <T extends StatusPacket<T>> void copyData(StatusPacket<?> statusMessage)
         {
            ConcurrentRingBuffer<T> buffer = (ConcurrentRingBuffer<T>) statusMessageClassToBufferMap.get(statusMessage.getClass());
            T next = buffer.next();
            if (next != null)
            {
               next.set((T) statusMessage);
               buffer.commit();
            }
         }
      };
      controllerStatusOutputManager.attachGlobalStatusMessageListener(globalStatusMessageListener);
   }

   @Override
   public void run()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         ConcurrentRingBuffer<? extends StatusPacket<?>> buffer = statusMessageClassToBufferMap.get(listOfSupportedStatusMessages.get(i));
         if (buffer.poll())
         {
            StatusPacket<?> statusMessage;
            while ((statusMessage = buffer.read()) != null)
            {
               packetCommunicator.send(statusMessage);
            }
            buffer.flush();
         }
      }
   }

   @Override
   public void closeAndDispose()
   {
      scheduler.shutdown();
   }
}
