package us.ihmc.quadrupedCommunication.networkProcessing;

import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import ihmc_common_msgs.msg.dds.MessageCollection;
import ihmc_common_msgs.msg.dds.MessageCollectionNotification;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.commons.PrintTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools.MessageUnpacker;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The NetworkSubscriber is meant to used as a generic interface between a network packet
 * communicator and the module API. It automatically creates all the {@link PacketConsumer} for
 * all the messages supported by the {@link CommandInputManager}. The status messages are send to
 * the network communicator on a separate thread to avoid any delay in the module thread.
 *
 * @author Robert
 */
public class NetworkSubscriber
{
   private static final boolean DEBUG = false;

   /** The input API to which the received messages should be submitted. */
   private final CommandInputManager controllerCommandInputManager;
   /** The output API that provides the status messages to send to the packet communicator. */
   private final OutputManager messageOutputManager;
   /** Used to filter messages coming in. */
   private final AtomicReference<MessageFilter> messageFilter;
   /** Used to filter messages coming in and report an error. */
   private final AtomicReference<MessageValidator> messageValidator;
   /** Used to synchronize the execution of a message collection. */
   private final List<MessageCollector> messageCollectors = new ArrayList<>();

   /** All the possible output message that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedOutputMessages;

   /** All the possible messages that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedControlMessages;

   /**
    * Local buffers for each message to ensure proper copying from the controller thread to the
    * communication thread.
    */
   private final Map<Class<? extends Settable<?>>, ROS2PublisherBasics<?>> outputMessagePublisherMap = new HashMap<>();

   private final RealtimeROS2Node realtimeROS2Node;

   private final ROS2Topic inputTopic;

   public NetworkSubscriber(ROS2Topic inputTopic, CommandInputManager controllerCommandInputManager,
                            ROS2Topic outputTopic, OutputManager messageOutputManager, RealtimeROS2Node realtimeROS2Node)
   {
      this.inputTopic = inputTopic;
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.messageOutputManager = messageOutputManager;
      this.realtimeROS2Node = realtimeROS2Node;
      listOfSupportedOutputMessages = messageOutputManager.getListOfSupportedMessages();
      listOfSupportedControlMessages = controllerCommandInputManager.getListOfSupportedMessages();

      messageFilter = new AtomicReference<>(message -> true);
      messageValidator = new AtomicReference<>(message -> null);

      if (realtimeROS2Node == null)
         PrintTools.error(this, "No ROS2 node, " + getClass().getSimpleName() + " cannot be created.");

      messageOutputManager.registerOutputMessage(InvalidPacketNotificationPacket.class, outputTopic);

      createPublishersSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
   }

   @SuppressWarnings({"unused", "unchecked"})
   private static <T> void getMessageTopicDataType(T messageInstance, Map<Class<?>, TopicDataType<?>> mapToModify)
   {
      Class<T> messageType = (Class<T>) messageInstance.getClass();

      if (mapToModify.containsKey(messageType))
         return;

      TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);

      mapToModify.put(messageType, topicDataType);
   }

   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType, int expectedMessageSize,
                                                                            MessageUnpacker<T> messageUnpacker)
   {
      registerSubcriberWithMessageUnpacker(multipleMessageType, inputTopic, expectedMessageSize, messageUnpacker);
   }

   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                            ROS2Topic inputTopic, int expectedMessageSize,
                                                                            MessageUnpacker<T> messageUnpacker)
   {
      final List<Settable<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);

      ROS2Topic topicName = inputTopic.withTypeName(multipleMessageType);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, multipleMessageType, topicName,
                                           s -> unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, s.takeNextData()));
   }

   private <T extends Settable<T>> void unpackMultiMessage(Class<T> multipleMessageHolderClass, MessageUnpacker<T> messageUnpacker,
                                                           List<Settable<?>> unpackedMessages, T multipleMessageHolder)
   {
      if (DEBUG)
         PrintTools.debug(NetworkSubscriber.this,
                          "Received message: " + multipleMessageHolder.getClass().getSimpleName() + ", " + multipleMessageHolder);

      String errorMessage = messageValidator.get().validate(multipleMessageHolder);

      if (errorMessage != null)
      {
         reportInvalidMessage(multipleMessageHolderClass, errorMessage);
         return;
      }

      if (testMessageWithMessageFilter(multipleMessageHolder))
      {
         messageUnpacker.unpackMessage(multipleMessageHolder, unpackedMessages);

         for (int i = 0; i < unpackedMessages.size(); i++)
         {
            receivedMessage(unpackedMessages.get(i));
         }
         unpackedMessages.clear();
      }
   }

   public void addMessageCollector(MessageIDExtractor messageIDExtractor, ROS2Topic messageTopicName)
   {
      addMessageCollectors(messageIDExtractor, 1, messageTopicName);
   }

   public void addMessageCollectors(MessageIDExtractor messageIDExtractor, int numberOfSimultaneousCollectionsToSupport, ROS2Topic messageTopicName)
   {
      ROS2PublisherBasics<MessageCollectionNotification> publisher = createPublisher(MessageCollectionNotification.class, messageTopicName);
      listOfSupportedOutputMessages.add(MessageCollectionNotification.class);

      for (int i = 0; i < numberOfSimultaneousCollectionsToSupport; i++)
      {
         messageCollectors.add(new MessageCollector(messageIDExtractor, listOfSupportedControlMessages));
      }

      MessageCollection messageCollection = new MessageCollection();

      ROS2Topic topicName = inputTopic.withTypeName(MessageCollection.class);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, MessageCollection.class, topicName, s -> {
         s.takeNextData(messageCollection, null);

         for (int i = 0; i < numberOfSimultaneousCollectionsToSupport; i++)
         {
            MessageCollector collector = messageCollectors.get(i);

            if (!collector.isCollecting())
            {
               publisher.publish(collector.startCollecting(messageCollection));
               return;
            }
         }

         PrintTools.warn("No collector available to process the MessageCollection with ID: " + messageCollection.getSequenceId());
      });
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
   private <T extends Settable<T>> void createPublishersSubscribersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedOutputMessages.size(); i++)
      {
         Class<T> messageClass = (Class<T>) listOfSupportedOutputMessages.get(i);
         outputMessagePublisherMap.put(messageClass, createPublisher(messageClass, messageOutputManager.getMessageTopicName(messageClass)));
      }

      for (int i = 0; i < listOfSupportedControlMessages.size(); i++)
      { // Creating the subscribers
         Class<T> messageClass = (Class<T>) listOfSupportedControlMessages.get(i);
         T messageLocalInstance = ROS2TopicNameTools.newMessageInstance(messageClass);
         ROS2Topic topicName = inputTopic.withTypeName(messageClass);

         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, messageClass, topicName, s -> {
            s.takeNextData(messageLocalInstance, null);
            receivedMessage(messageLocalInstance);
         });
      }
   }

   private <T extends Settable<T>> ROS2PublisherBasics<T> createPublisher(Class<T> messageClass, ROS2Topic topicName)
   {
      ROS2PublisherBasics<T> publisher = realtimeROS2Node.createPublisher(topicName.withTypeName(messageClass));
      return publisher;
   }

   @SuppressWarnings("unchecked")
   private <T extends Settable<T>> void receivedMessage(Settable<?> message)
   {
      if (DEBUG)
         PrintTools.debug(NetworkSubscriber.this, "Received message: " + message.getClass().getSimpleName() + ", " + message);

      for (int collectorIndex = 0; collectorIndex < messageCollectors.size(); collectorIndex++)
      {
         MessageCollector messageCollector = messageCollectors.get(collectorIndex);

         if (messageCollector.isCollecting() && messageCollector.interceptMessage(message))
         {
            if (DEBUG)
               PrintTools.debug(NetworkSubscriber.this, "Collecting message: " + message.getClass().getSimpleName() + ", " + message);

            if (!messageCollector.isCollecting())
            {
               List<Settable<?>> collectedMessages = messageCollector.getCollectedMessages();
               for (int i = 0; i < collectedMessages.size(); i++)
               {
                  receivedMessage(collectedMessages.get(i));
               }
               messageCollector.reset();
            }

            return;
         }
      }

      String errorMessage = messageValidator.get().validate(message);

      if (errorMessage != null)
      {
         reportInvalidMessage((Class<? extends Settable<?>>) message.getClass(), errorMessage);
         return;
      }

      if (testMessageWithMessageFilter(message))
         controllerCommandInputManager.submitMessage((T) message);
   }

   private boolean testMessageWithMessageFilter(Settable<?> messageToTest)
   {
      if (!messageFilter.get().isMessageValid(messageToTest))
      {
         if (DEBUG)
            PrintTools.error(NetworkSubscriber.this, "Packet failed to validate filter! Filter class: "
                  + messageFilter.get().getClass().getSimpleName() + ", rejected message: " + messageToTest.getClass().getSimpleName());
         return false;
      }
      return true;
   }

   private void reportInvalidMessage(Class<?> messageClass, String errorMessage)
   {
      publishStatusMessage(MessageTools.createInvalidPacketNotificationPacket(messageClass, errorMessage));
      PrintTools.error(NetworkSubscriber.this, "Packet failed to validate: " + messageClass.getSimpleName());
      PrintTools.error(NetworkSubscriber.this, errorMessage);
   }

   private void createGlobalStatusMessageListener()
   {
      messageOutputManager.attachGlobalOutputMessageListener(statusMessage -> publishStatusMessage(statusMessage));
   }

   @SuppressWarnings("unchecked")
   private <T> void publishStatusMessage(T message)
   {
      ROS2PublisherBasics<T> publisher = (ROS2PublisherBasics<T>) outputMessagePublisherMap.get(message.getClass());
      publisher.publish(message);
   }

   public static interface MessageFilter
   {
      public boolean isMessageValid(Object message);
   }

   public static interface MessageValidator
   {
      String validate(Object message);
   }
}
