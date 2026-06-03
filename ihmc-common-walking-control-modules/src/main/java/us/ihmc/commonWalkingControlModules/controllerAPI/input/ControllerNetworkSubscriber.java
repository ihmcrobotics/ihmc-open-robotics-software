package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import ihmc_common_msgs.MessageCollection;
import ihmc_common_msgs.MessageCollectionNotification;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools.MessageUnpacker;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The ControllerNetworkSubscriber is meant to used as a generic interface between a network packet
 * communicator and the controller API. It automatically creates ROS 2 subscriptions for
 * all the messages supported by the {@link CommandInputManager}. The status messages are send to
 * the network communicator on a separate thread to avoid any delay in the controller thread.
 *
 * @author Sylvain
 */
public class ControllerNetworkSubscriber
{
   private static final double LOW_FREQUENCY_PUBLISH_RATE = 50.0;
   private static final boolean DEBUG = false;

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
   private final List<Class<? extends ROS2Message<?>>> listOfSupportedStatusMessages;

   /** All the possible messages that can be sent to the communicator. */
   private final List<Class<? extends ROS2Message<?>>> listOfSupportedControlMessages;

   /**
    * Local buffers for each message to ensure proper copying from the controller thread to the
    * communication thread.
    */
   private final Map<Class<? extends ROS2Message<?>>, ROS2Publisher<?>> statusMessagePublisherMap = new HashMap<>();
   private final Map<Class<? extends ROS2Message<?>>, ThrottledROS2Publisher> lowFrequencyStatusMessagePublisherMap = new HashMap<>();

   private final ROS2Node ros2Node;

   private final ROS2Topic<?> inputTopic;
   private final ROS2Topic<?> outputTopic;

   public ControllerNetworkSubscriber(ROS2Topic<?> inputTopic,
                                      CommandInputManager controllerCommandInputManager,
                                      ROS2Topic<?> outputTopic,
                                      StatusMessageOutputManager controllerStatusOutputManager,
                                      ROS2Node ros2Node)
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

      createPublishersSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
   }

   public <T extends ROS2Message<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                             int expectedMessageSize,
                                                                             MessageUnpacker<T> messageUnpacker)
   {
      final List<ROS2Message<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);

      T localInstance = ROS2Message.createInstance(multipleMessageType);
      ros2Node.createSubscription(ControllerAPI.getTopic(inputTopic, multipleMessageType), reader ->
      {
         reader.read(localInstance);
         unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, localInstance);
      });
   }

   private <T extends ROS2Message<T>> void unpackMultiMessage(Class<T> multipleMessageHolderClass,
                                                           MessageUnpacker<T> messageUnpacker,
                                                           List<ROS2Message<?>> unpackedMessages,
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
      @SuppressWarnings("unchecked")
      ROS2Publisher<MessageCollectionNotification> publisher = (ROS2Publisher<MessageCollectionNotification>) statusMessagePublisherMap.computeIfAbsent(MessageCollectionNotification.class,
                                                                                                                                                        messageClass -> createPublisher(messageClass));

      for (int i = 0; i < numberOfSimultaneousCollectionsToSupport; i++)
      {
         messageCollectors.add(new MessageCollector(messageIDExtractor, listOfSupportedControlMessages));
      }

      MessageCollection messageCollection = new MessageCollection();

      ros2Node.createSubscription(ControllerAPI.getTopic(inputTopic, MessageCollection.class), reader ->
      {
         reader.read(messageCollection);

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
   private void createPublishersSubscribersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         Class<? extends ROS2Message<?>> messageClass = listOfSupportedStatusMessages.get(i);
         statusMessagePublisherMap.put(messageClass, createPublisher((Class<? extends ROS2Message<?>>) messageClass));
         lowFrequencyStatusMessagePublisherMap.put(messageClass, createLowFrequencyPublisher((Class<? extends ROS2Message<?>>) messageClass));
      }

      for (int i = 0; i < listOfSupportedControlMessages.size(); i++)
      { // Creating the subscribers
         Class<? extends ROS2Message<?>> messageClass = (Class<? extends ROS2Message<?>>) listOfSupportedControlMessages.get(i);
         @SuppressWarnings({"unchecked", "rawtypes"})
         Class messageClassRaw = messageClass;

         ros2Node.createSubscription(ControllerAPI.getTopic(inputTopic, messageClassRaw), reader ->
         {
            ROS2Message<?> message = reader.read();
            if (message != null)
               receivedMessage(message);
         });
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private ROS2Publisher<?> createPublisher(Class<? extends ROS2Message<?>> messageClass)
   {
      return ros2Node.createPublisher(ControllerAPI.getTopic(outputTopic, (Class) messageClass));
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private ThrottledROS2Publisher createLowFrequencyPublisher(Class<? extends ROS2Message<?>> messageClass)
   {
      return new ThrottledROS2Publisher(ros2Node.createPublisher(ControllerAPI.getLowFrequencyTopic(outputTopic, (Class) messageClass)),
                                       LOW_FREQUENCY_PUBLISH_RATE);
   }

   @SuppressWarnings("unchecked")
   private <T extends ROS2Message<T>> void receivedMessage(ROS2Message<?> message)
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
               List<ROS2Message<?>> collectedMessages = messageCollector.getCollectedMessages();
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
         reportInvalidMessage((Class<? extends ROS2Message<?>>) message.getClass(), errorMessage);
         return;
      }

      if (testMessageWithMessageFilter(message))
         controllerCommandInputManager.submitMessage((T) message);
   }

   private boolean testMessageWithMessageFilter(ROS2Message<?> messageToTest)
   {
      if (!messageFilter.get().isMessageValid(messageToTest))
      {
         if (DEBUG)
            LogTools.error("Message failed to validate filter! Filter class: {}, rejected message: {}",
                           messageFilter.get().getClass().getSimpleName(),
                           messageToTest.getClass().getSimpleName());
         return false;
      }
      return true;
   }

   private void reportInvalidMessage(Class<?> messageClass, String errorMessage)
   {
      publishStatusMessage(MessageTools.createInvalidPacketNotificationPacket(messageClass, errorMessage));
      LogTools.error("Message failed to validate: {}", messageClass.getSimpleName());
      LogTools.error(errorMessage);
   }

   private void createGlobalStatusMessageListener()
   {
      controllerStatusOutputManager.attachGlobalStatusMessageListener(statusMessage -> publishStatusMessage(statusMessage));
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private void publishStatusMessage(ROS2Message<?> message)
   {
      ROS2Publisher publisher = (ROS2Publisher) statusMessagePublisherMap.get(message.getClass());
      publisher.publish((ROS2Message) message);

      ThrottledROS2Publisher throttledPublisher = lowFrequencyStatusMessagePublisherMap.get(message.getClass());
      throttledPublisher.publish((ROS2Message<?>) message);
   }

   public static interface MessageFilter
   {
      public boolean isMessageValid(Object message);
   }

   public static interface MessageValidator
   {
      String validate(Object message);
   }

   private static class ThrottledROS2Publisher
   {
      private final ROS2Publisher<?> publisher;
      private final Throttler throttler = new Throttler();

      public ThrottledROS2Publisher(ROS2Publisher<?> publisher, double maxPublishFrequency)
      {
         this.publisher = publisher;
         throttler.setFrequency(maxPublishFrequency);
      }

      @SuppressWarnings({"unchecked", "rawtypes"})
      public void publish(ROS2Message<?> message)
      {
         if (throttler.run())
            ((ROS2Publisher) publisher).publish(message);
      }
   }
}
