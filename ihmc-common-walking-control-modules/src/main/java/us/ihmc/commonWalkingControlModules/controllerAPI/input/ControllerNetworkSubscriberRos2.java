package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.MessageCollectionNotification;
import controller_msgs.msg.dds.MessageCollectionPubSubType;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageValidator;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools.MessageUnpacker;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.RealtimeRos2Publisher;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.util.PeriodicThreadScheduler;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The ControllerNetworkSubscriber is meant to used as a generic interface between a network packet
 * communicator and the controller API. It automatically creates all the {@link PacketConsumer} for
 * all the messages supported by the {@link CommandInputManager}. The status messages are send to
 * the network communicator on a separate thread to avoid any delay in the controller thread.
 *
 * @author Sylvain
 *
 */
public class ControllerNetworkSubscriberRos2 implements Runnable, CloseableAndDisposable
{
   private static final boolean DEBUG = false;

   private final int buffersCapacity = 16;
   /** The input API to which the received messages should be submitted. */
   private final CommandInputManager controllerCommandInputManager;
   /** The output API that provides the status messages to send to the packet communicator. */
   private final StatusMessageOutputManager controllerStatusOutputManager;
   /** Topic name for communication */
   private final String topicName;
   /** ROS2 node for publishing and subscribing */
   private final RealtimeRos2Node realtimeRos2Node;
   /** Publishers for ROS2 */
   private final HashMap<String, RealtimeRos2Publisher<Packet>> publishers;
   /** Used to schedule status message sending. */
   private final PeriodicThreadScheduler scheduler;
   /** Used to filter messages coming in. */
   private final AtomicReference<MessageFilter> messageFilter;
   /** Used to filter messages coming in and report an error. */
   private final AtomicReference<MessageValidator> messageValidator;
   /** Used to synchronize the execution of a message collection. */
   private MessageCollector messageCollector = MessageCollector.createDummyCollector();

   /** All the possible status message that can be sent to the communicator. */
   private final List<Class<? extends Packet<?>>> listOfSupportedStatusMessages;

   /** All the possible messages that can be sent to the communicator. */
   private final List<Class<? extends Packet<?>>> listOfSupportedControlMessages;

   /**
    * Local buffers for each message to ensure proper copying from the controller thread to the
    * communication thread.
    */
   private final Map<Class<? extends Packet<?>>, ConcurrentRingBuffer<? extends Packet<?>>> statusMessageClassToBufferMap = new HashMap<>();

   public ControllerNetworkSubscriberRos2(CommandInputManager controllerCommandInputManager, StatusMessageOutputManager controllerStatusOutputManager,
                                          PeriodicThreadScheduler scheduler, RealtimeRos2Node realtimeRos2Node)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.scheduler = scheduler;
      this.realtimeRos2Node = realtimeRos2Node;
      publishers = new HashMap<>();
      topicName = "/controller_network_subscriber";
      listOfSupportedStatusMessages = controllerStatusOutputManager.getListOfSupportedMessages();
      listOfSupportedControlMessages = controllerCommandInputManager.getListOfSupportedMessages();
      messageFilter = new AtomicReference<>(message -> true);
      messageValidator = new AtomicReference<>(message -> null);

      if (realtimeRos2Node == null)
      {
         PrintTools.error(this, "No ros2 node, " + getClass().getSimpleName() + " cannot be created.");
         return;
      }

      listOfSupportedStatusMessages.add(InvalidPacketNotificationPacket.class);

      createAllSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
      createAllStatusMessageBuffers();

      if (scheduler != null)
         scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
   }

   public <T extends Packet<T>> void registerSubcriberWithMessageUnpacker(Class<T> messageType, int expectedMessageSize,
                                                                          MessageUnpacker<T> messageUnpacker)
   {
      try
      {
         final List<Packet<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);
         final T message = messageType.newInstance();
         realtimeRos2Node.createCallbackSubscription(message.getPubSubTypePacket().get(), topicName, subscriber -> {
            try
            {
               subscriber.takeNextData(message, null);
               if (DEBUG)
                  PrintTools.debug(ControllerNetworkSubscriberRos2.this,
                                   "Received message: " + messageType.getName() + ", " + message);

               String errorMessage = messageValidator.get().validate(message);

               if (errorMessage != null)
               {
                  reportInvalidMessage(message.getClass(), errorMessage);
                  return;
               }

               if (testMessageWithMessageFilter(message))
               {
                  messageUnpacker.unpackMessage(message, unpackedMessages);

                  for (int i = 0; i < unpackedMessages.size(); i++)
                  {
                     receivedMessage(unpackedMessages.get(i));
                  }
                  unpackedMessages.clear();
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         });
         publishers.put(message.getClass().getName(), realtimeRos2Node.createPublisher(message.getPubSubTypePacket().get(), topicName));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         e.printStackTrace();
      }
   }

   public void addMessageCollector(MessageIDExtractor messageIDExtractor)
   {
      messageCollector = new MessageCollector(messageIDExtractor, listOfSupportedControlMessages);
      createStatusMessageBuffer(MessageCollectionNotification.class);
      listOfSupportedStatusMessages.add(MessageCollectionNotification.class);
      try
      {
         final MessageCollection message = new MessageCollection();
         realtimeRos2Node.createCallbackSubscription(new MessageCollectionPubSubType(), topicName, subscriber -> {
            try
            {
               subscriber.takeNextData(message, null);
               MessageCollectionNotification notification = messageCollector.startCollecting(message);
               copyAndCommitStatusMessage(notification);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         });
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void addMessageFilter(MessageFilter newFilter)
   {
      messageFilter.set(newFilter);
   }

   public void removeMessageFilter()
   {
      messageFilter.set(null);
   }

   public void addMessageValidator(MessageValidator newValidator)
   {
      messageValidator.set(newValidator);
   }

   public void removeMessageValidator()
   {
      messageValidator.set(null);
   }

   @SuppressWarnings("unchecked")
   private <T extends Packet<T>> void createAllStatusMessageBuffers()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         createStatusMessageBuffer((Class<T>) listOfSupportedStatusMessages.get(i));
      }
   }

   private <T extends Packet<T>> void createStatusMessageBuffer(Class<T> statusMessageClass)
   {
      Builder<T> builder = CommandInputManager.createBuilderWithEmptyConstructor(statusMessageClass);
      ConcurrentRingBuffer<T> newBuffer = new ConcurrentRingBuffer<>(builder, buffersCapacity);
      statusMessageClassToBufferMap.put(statusMessageClass, newBuffer);
   }

   @SuppressWarnings("unchecked")
   private <T extends Packet<T>> void createAllSubscribersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedControlMessages.size(); i++)
      {
         try
         {
            Class<T> messageClass = (Class<T>) listOfSupportedControlMessages.get(i);
            final T message = messageClass.newInstance();
            realtimeRos2Node.createCallbackSubscription(message.getPubSubTypePacket().get(), topicName, subscriber -> {
               try
               {
                  subscriber.takeNextData(message, null);
                  receivedMessage(message);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            });
            publishers.put(messageClass.getName(), realtimeRos2Node.createPublisher(message.getPubSubTypePacket().get(), topicName));
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
         catch (InstantiationException | IllegalAccessException e)
         {
            e.printStackTrace();
         }
      }
   }

   @SuppressWarnings("unchecked")
   private <T extends Packet<T>> void receivedMessage(Packet<?> message)
   {
      if (DEBUG)
         PrintTools.debug(ControllerNetworkSubscriberRos2.this, "Received message: " + message.getClass().getSimpleName() + ", " + message);

      if (messageCollector.isCollecting() && messageCollector.interceptMessage(message))
      {
         if (DEBUG)
            PrintTools.debug(ControllerNetworkSubscriberRos2.this, "Collecting message: " + message.getClass().getSimpleName() + ", " + message);

         if (!messageCollector.isCollecting())
         {
            List<Packet<?>> collectedMessages = messageCollector.getCollectedMessages();
            for (int i = 0; i < collectedMessages.size(); i++)
            {
               receivedMessage(collectedMessages.get(i));
            }
            messageCollector.reset();
         }

         return;
      }

      String errorMessage = messageValidator.get().validate(message);

      if (errorMessage != null)
      {
         reportInvalidMessage((Class<? extends Packet<?>>) message.getClass(), errorMessage);
         return;
      }

      if (testMessageWithMessageFilter(message))
         controllerCommandInputManager.submitMessage((T) message);
   }

   private boolean testMessageWithMessageFilter(Packet<?> messageToTest)
   {
      if (!messageFilter.get().isMessageValid(messageToTest))
      {
         if (DEBUG)
            PrintTools.error(ControllerNetworkSubscriberRos2.this,
                             "Packet failed to validate filter! Filter class: "
                                   + messageFilter.get().getClass().getSimpleName() + ", rejected message: " + messageToTest.getClass().getSimpleName());
         return false;
      }
      return true;
   }

   private void reportInvalidMessage(Class<? extends Packet> messageClass, String errorMessage)
   {
      ConcurrentRingBuffer<?> buffer = statusMessageClassToBufferMap.get(InvalidPacketNotificationPacket.class);

      InvalidPacketNotificationPacket next = (InvalidPacketNotificationPacket) buffer.next();

      if (next != null)
      {
         next.setPacketClassSimpleName(messageClass.getSimpleName());
         next.setErrorMessage(errorMessage);
         buffer.commit();
      }

      PrintTools.error(ControllerNetworkSubscriberRos2.this, "Packet failed to validate: " + messageClass.getSimpleName());
      PrintTools.error(ControllerNetworkSubscriberRos2.this, errorMessage);
   }

   private void createGlobalStatusMessageListener()
   {
      controllerStatusOutputManager.attachGlobalStatusMessageListener(statusMessage -> copyAndCommitStatusMessage(statusMessage));
   }

   @SuppressWarnings("unchecked")
   private <T extends Packet<T>> void copyAndCommitStatusMessage(Packet<?> statusMessage)
   {
      ConcurrentRingBuffer<T> buffer = (ConcurrentRingBuffer<T>) statusMessageClassToBufferMap.get(statusMessage.getClass());
      T next = buffer.next();
      if (next != null)
      {
         next.set((T) statusMessage);
         buffer.commit();
      }
   }

   @Override
   public void run()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         ConcurrentRingBuffer<? extends Packet<?>> buffer = statusMessageClassToBufferMap.get(listOfSupportedStatusMessages.get(i));
         if (buffer.poll())
         {
            Packet<?> statusMessage;
            while ((statusMessage = buffer.read()) != null)
            {
               publishers.get(statusMessage.getClass().getName()).publish(statusMessage);
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
