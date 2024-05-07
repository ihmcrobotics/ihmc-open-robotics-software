package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import ihmc_common_msgs.msg.dds.MessageCollection;
import ihmc_common_msgs.msg.dds.MessageCollectionNotification;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools.MessageUnpacker;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;

/**
 * The ControllerNetworkSubscriber is meant to used as a generic interface between a network packet
 * communicator and the controller API. It automatically creates all the {@link PacketConsumer} for
 * all the messages supported by the {@link CommandInputManager}. The status messages are send to
 * the network communicator on a separate thread to avoid any delay in the controller thread.
 *
 * @author Sylvain
 */
public class ControllerNetworkSubscriber
{
   private static final boolean DEBUG = true;

   /** The input API to which the received messages should be submitted. */
   private final CommandInputManager controllerCommandInputManager;
   /** The output API that provides the status messages to send to the packet communicator. */
   private final StatusMessageOutputManager controllerStatusOutputManager;
   /** Used to filter messages coming in. */
   private final AtomicReference<MessageFilter> messageFilter;
   /** Used to filter messages coming in and report an error. */
   private final AtomicReference<MessageValidator> messageValidator;
   /** Used to synchronize the execution of a message collection. */
   private final List<MessageCollector> messageCollectors = new ArrayList<>();

   /** All the possible status message that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedStatusMessages;

   /** All the possible messages that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedControlMessages;

   /**
    * Local buffers for each message to ensure proper copying from the controller thread to the
    * communication thread.
    */
   private final Map<Class<? extends Settable<?>>, ROS2PublisherBasics<?>> statusMessagePublisherMap = new HashMap<>();

   private final ROS2NodeInterface ros2Node;

   private final ROS2Topic<?> inputTopic;
   private final ROS2Topic<?> outputTopic;

   public ControllerNetworkSubscriber(ROS2Topic<?> inputTopic,
                                      CommandInputManager controllerCommandInputManager,
                                      ROS2Topic<?> outputTopic,
                                      StatusMessageOutputManager controllerStatusOutputManager,
                                      ROS2NodeInterface ros2Node)
   {
      this.inputTopic = inputTopic;
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.outputTopic = outputTopic;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.ros2Node = ros2Node;
      listOfSupportedStatusMessages = controllerStatusOutputManager.getListOfSupportedMessages();
      listOfSupportedControlMessages = controllerCommandInputManager.getListOfSupportedMessages();

      messageFilter = new AtomicReference<>(message -> true);
      messageValidator = new AtomicReference<>(message -> null);

      if (ros2Node == null)
         LogTools.error("No ROS2 node, {} cannot be created.", getClass().getSimpleName());

      listOfSupportedStatusMessages.add(InvalidPacketNotificationPacket.class);

      createPublishersSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
   }

   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                            int expectedMessageSize,
                                                                            MessageUnpacker<T> messageUnpacker)
   {
      final List<Settable<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);

      try
      {
         T localInstance = multipleMessageType.newInstance();
         NewMessageListener<T> messageListener = s ->
         {
            s.takeNextData(localInstance, null);
            unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, localInstance);
         };

         ros2Node.createSubscription(ControllerAPI.getTopic(inputTopic, multipleMessageType), messageListener);
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   private <T extends Settable<T>> void unpackMultiMessage(Class<T> multipleMessageHolderClass,
                                                           MessageUnpacker<T> messageUnpacker,
                                                           List<Settable<?>> unpackedMessages,
                                                           T multipleMessageHolder)
   {
      if (DEBUG)
         LogTools.debug("Received message: {}, {}.", multipleMessageHolder.getClass().getSimpleName(), multipleMessageHolder);

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

   public void addMessageCollector(MessageIDExtractor messageIDExtractor)
   {
      addMessageCollectors(messageIDExtractor, 1);
   }

   public void addMessageCollectors(MessageIDExtractor messageIDExtractor, int numberOfSimultaneousCollectionsToSupport)
   {
      ROS2PublisherBasics<MessageCollectionNotification> publisher = createPublisher(MessageCollectionNotification.class);
      listOfSupportedStatusMessages.add(MessageCollectionNotification.class);

      for (int i = 0; i < numberOfSimultaneousCollectionsToSupport; i++)
      {
         messageCollectors.add(new MessageCollector(messageIDExtractor, listOfSupportedControlMessages));
      }

      MessageCollection messageCollection = new MessageCollection();

      ros2Node.createSubscription(ControllerAPI.getTopic(inputTopic, MessageCollection.class), s ->
      {
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

         LogTools.warn("No collector available to process the MessageCollection with ID: {}", messageCollection.getSequenceId());
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
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         Class<T> messageClass = (Class<T>) listOfSupportedStatusMessages.get(i);
         statusMessagePublisherMap.put(messageClass, createPublisher(messageClass));
      }

      for (int i = 0; i < listOfSupportedControlMessages.size(); i++)
      { // Creating the subscribers
         Class<T> messageClass = (Class<T>) listOfSupportedControlMessages.get(i);
         T messageLocalInstance = ROS2TopicNameTools.newMessageInstance(messageClass);

         ros2Node.createSubscription(ControllerAPI.getTopic(inputTopic, messageClass), s ->
         {
            s.takeNextData(messageLocalInstance, null);
            receivedMessage(messageLocalInstance);
         });
      }
   }

   private <T extends Settable<T>> ROS2PublisherBasics<T> createPublisher(Class<T> messageClass)
   {
      return ros2Node.createPublisher(ControllerAPI.getTopic(outputTopic, messageClass));
   }

   @SuppressWarnings("unchecked")
   private <T extends Settable<T>> void receivedMessage(Settable<?> message)
   {
      if (DEBUG)
         LogTools.debug("Received message: {}, {}", message.getClass().getSimpleName(), message);

      for (int collectorIndex = 0; collectorIndex < messageCollectors.size(); collectorIndex++)
      {
         MessageCollector messageCollector = messageCollectors.get(collectorIndex);

         if (messageCollector.isCollecting() && messageCollector.interceptMessage(message))
         {
            if (DEBUG)
               LogTools.debug("Collecting message: {}, {}", message.getClass().getSimpleName(), message);

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
            LogTools.error("Packet failed to validate filter! Filter class: {}, rejected message: {}",
                           messageFilter.get().getClass().getSimpleName(),
                           messageToTest.getClass().getSimpleName());
         return false;
      }
      return true;
   }

   private void reportInvalidMessage(Class<?> messageClass, String errorMessage)
   {
      publishStatusMessage(MessageTools.createInvalidPacketNotificationPacket(messageClass, errorMessage));
      LogTools.error("Packet failed to validate: {}", messageClass.getSimpleName());
      LogTools.error(errorMessage);
   }

   private void createGlobalStatusMessageListener()
   {
      controllerStatusOutputManager.attachGlobalStatusMessageListener(statusMessage -> publishStatusMessage(statusMessage));
   }

   @SuppressWarnings("unchecked")
   private <T> void publishStatusMessage(T message)
   {
      ROS2PublisherBasics<T> publisher = (ROS2PublisherBasics<T>) statusMessagePublisherMap.get(message.getClass());
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
