package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.GlobalStatusMessageListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;

public class ControllerNetworkSubscriber implements Runnable, CloseableAndDisposable
{
   private final int buffersCapacity = 16;
   private final CommandInputManager controllerCommandInputManager;
   private final StatusMessageOutputManager controllerStatusOutputManager;
   private final HumanoidGlobalDataProducer globalDataProducer;
   private final PeriodicThreadScheduler scheduler;

   private final List<Class<? extends StatusPacket<?>>> listOfSupportedMessages;
   private final Map<Class<? extends StatusPacket<?>>, ConcurrentRingBuffer<? extends StatusPacket<?>>> statusMessageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends StatusPacket<?>>, Builder<? extends StatusPacket<?>>> statusMessageClassToBuilderMap = new HashMap<>();

   public ControllerNetworkSubscriber(CommandInputManager controllerCommandInputManager, StatusMessageOutputManager controllerStatusOutputManager,
         CloseableAndDisposableRegistry closeAndDisposeRegistry, PeriodicThreadScheduler scheduler, HumanoidGlobalDataProducer globalDataProducer)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.scheduler = scheduler;
      this.globalDataProducer = globalDataProducer;
      listOfSupportedMessages = controllerStatusOutputManager.getListOfSupportedMessages();

      createAllSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
      createAllStatusMessageBuffers();

      scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
      closeAndDisposeRegistry.registerCloseableAndDisposable(this);
   }

   @SuppressWarnings("unchecked")
   private <T extends StatusPacket<T>> void createAllStatusMessageBuffers()
   {
      for (int i = 0; i < listOfSupportedMessages.size(); i++)
      {
         Class<T> statusMessageClass = (Class<T>) listOfSupportedMessages.get(i);
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
      for (int i = 0; i < listOfSupportedMessages.size(); i++)
      {
         ConcurrentRingBuffer<? extends StatusPacket<?>> buffer = statusMessageClassToBufferMap.get(listOfSupportedMessages.get(i));
         if (buffer.poll())
         {
            StatusPacket<?> statusMessage;
            while ((statusMessage = buffer.read()) != null)
            {
               if (statusMessage instanceof CapturabilityBasedStatus)
                  globalDataProducer.send((CapturabilityBasedStatus) statusMessage);
               else
               {
                  cloneAndSend(statusMessage);
               }
                  
            }
            buffer.flush();
         }
      }
   }

   @SuppressWarnings("unchecked")
   public <T extends StatusPacket<T>> void cloneAndSend(StatusPacket<?> statusMessage)
   {
      T statusMessageCopy = (T) statusMessageClassToBuilderMap.get(statusMessage.getClass()).newInstance();
      statusMessageCopy.set((T) statusMessage);
      globalDataProducer.queueDataToSend(statusMessageCopy);
   }

   @Override
   public void closeAndDispose()
   {
      scheduler.shutdown();
   }
}
