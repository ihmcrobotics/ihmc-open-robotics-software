package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

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
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.util.PeriodicThreadScheduler;

/**
 * The ControllerNetworkSubscriber is meant to used as a generic interface between a network packet communicator and the controller API.
 * It automatically creates all the {@link PacketConsumer} for all the messages supported by the {@link CommandInputManager}.
 * The status messages are send to the network communicator on a separate thread to avoid any delay in the controller thread.
 * @author Sylvain
 *
 */
public class ControllerNetworkSubscriber implements Runnable, CloseableAndDisposable
{
   private static final boolean DEBUG = false;

   private final int buffersCapacity = 16;
   /** The input API to which the received messages should be submitted. */
   private final CommandInputManager controllerCommandInputManager;
   /** The output API that provides the status messages to send to the packet communicator. */
   private final StatusMessageOutputManager controllerStatusOutputManager;
   /** Communicator from which commands are received and status messages can send to. */
   private final PacketCommunicator packetCommunicator;
   /** Used to schedule status message sending. */
   private final PeriodicThreadScheduler scheduler;
   /** Used to filter messages coming in. */
   private final AtomicReference<MessageFilter> messageFilter = new AtomicReference<>(null);

   /** All the possible status message that can be sent to the communicator. */
   private final List<Class<? extends StatusPacket<?>>> listOfSupportedStatusMessages;
   /** Local buffers for each message to ensure proper copying from the controller thread to the communication thread. */
   private final Map<Class<? extends StatusPacket<?>>, ConcurrentRingBuffer<? extends StatusPacket<?>>> statusMessageClassToBufferMap = new HashMap<>();

   public ControllerNetworkSubscriber(CommandInputManager controllerCommandInputManager, StatusMessageOutputManager controllerStatusOutputManager,
         PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.scheduler = scheduler;
      this.packetCommunicator = packetCommunicator;
      listOfSupportedStatusMessages = controllerStatusOutputManager.getListOfSupportedMessages();

      if (packetCommunicator == null)
      {
         PrintTools.error(this, "No packet communicator, " + getClass().getSimpleName() + " cannot be created.");
         return;
      }

      listOfSupportedStatusMessages.add(InvalidPacketNotificationPacket.class);

      createAllSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
      createAllStatusMessageBuffers();

      if (scheduler != null)
         scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
   }

   public void addMessageFilter(MessageFilter newFilter)
   {
      messageFilter.set(newFilter);
   }

   public void removeMessageFilter()
   {
      messageFilter.set(null);
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

            if (DEBUG)
               PrintTools.debug(ControllerNetworkSubscriber.this, "Received message: " + message.getClass().getSimpleName() + ", " + message);

            if (errorMessage != null)
            {
               reportInvalidMessage(messageClass, errorMessage);
               PrintTools.error(ControllerNetworkSubscriber.this, "Packet failed to validate:"); 
               PrintTools.error(ControllerNetworkSubscriber.this, errorMessage); 
               return;
            }

            if (messageFilter.get() != null && !messageFilter.get().isMessageValid(message))
            {
               if (DEBUG)
                  PrintTools.error(ControllerNetworkSubscriber.this, "Packet failed to validate filter! Filter class: "
                        + messageFilter.get().getClass().getSimpleName() + ", rejected message: " + message.getClass().getSimpleName());
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
      if (scheduler != null)
         scheduler.shutdown();
   }

   public static interface MessageFilter
   {
      public boolean isMessageValid(Packet<?> message);
   }
}
