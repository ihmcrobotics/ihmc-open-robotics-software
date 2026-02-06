package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class StepGeneratorNetworkSubscriber
{
   private static final boolean DEBUG = false;

   /** The input API to which the received messages should be submitted. */
   private final CommandInputManager commandInputManager;

   private final StatusMessageOutputManager statusMessageOutputManager;

   /** Used to filter messages coming in. */
   private final AtomicReference<ControllerNetworkSubscriber.MessageFilter> messageFilter;
   /** Used to filter messages coming in and report an error. */
   private final AtomicReference<ControllerNetworkSubscriber.MessageValidator> messageValidator;

   /** All the possible messages that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedControlMessages;
   private final List<Class<? extends Settable<?>>> listOfSupportedStatusMessages;

   //
   private final Map<Class<? extends Settable<?>>, ROS2Publisher<?>> statusMessagePublisherMap = new HashMap<>();

   private final RealtimeROS2Node realtimeROS2Node;

   private final ROS2Topic<?> baseTopic;

   public StepGeneratorNetworkSubscriber(ROS2Topic<?> baseTopic,
                                         CommandInputManager csgCommandInputManager,
                                         StatusMessageOutputManager csgStatusMessageOutputManager,
                                         RealtimeROS2Node realtimeROS2Node)
   {
      this.baseTopic = baseTopic;
      this.commandInputManager = csgCommandInputManager;
      this.statusMessageOutputManager = csgStatusMessageOutputManager;
      this.realtimeROS2Node = realtimeROS2Node;

      listOfSupportedControlMessages = csgCommandInputManager.getListOfSupportedMessages();
      listOfSupportedStatusMessages = csgStatusMessageOutputManager.getListOfSupportedMessages();

      messageFilter = new AtomicReference<>(message -> true);
      messageValidator = new AtomicReference<>(message -> null);

      if (realtimeROS2Node == null)
         LogTools.error("No ROS2 node, {} cannot be created.", getClass().getSimpleName());

      createSubscribersForSupportedMessages();
      createPublishersForSupportedMessages();
   }

   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                            int expectedMessageSize,
                                                                            MessageUnpackingTools.MessageUnpacker<T> messageUnpacker)
   {
      registerSubcriberWithMessageUnpacker(multipleMessageType, baseTopic, expectedMessageSize, messageUnpacker);
   }

   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                            ROS2Topic<?> inputTopic,
                                                                            int expectedMessageSize,
                                                                            MessageUnpackingTools.MessageUnpacker<T> messageUnpacker)
   {
      registerSubcriberWithMessageUnpacker(multipleMessageType, inputTopic, null, expectedMessageSize, messageUnpacker);
   }

   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                            ROS2Topic<?> inputTopic,
                                                                            ROS2QosProfile qosProfile,
                                                                            int expectedMessageSize,
                                                                            MessageUnpackingTools.MessageUnpacker<T> messageUnpacker)
   {
      final List<Settable<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);

      ROS2Topic<T> topicName = inputTopic.withTypeName(multipleMessageType);
      try
      {
         T localInstance = multipleMessageType.newInstance();
         NewMessageListener<T> messageListener = s ->
         {
            s.takeNextData(localInstance, null);
            unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, localInstance);
         };

         if (qosProfile != null)
            topicName = topicName.withQoS(qosProfile);

         realtimeROS2Node.createSubscription(topicName, messageListener);
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   private <T extends Settable<T>> void unpackMultiMessage(Class<T> multipleMessageHolderClass,
                                                           MessageUnpackingTools.MessageUnpacker<T> messageUnpacker,
                                                           List<Settable<?>> unpackedMessages,
                                                           T multipleMessageHolder)
   {
      if (DEBUG)
         LogTools.debug("Received message: {}, {}.", multipleMessageHolder.getClass().getSimpleName(), multipleMessageHolder);

      String errorMessage = messageValidator.get().validate(multipleMessageHolder);

      if (errorMessage != null)
         return;

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


   public void addMessageFilter(ControllerNetworkSubscriber.MessageFilter newFilter)
   {
      messageFilter.set(newFilter);
   }

   public void removeMessageFilter()
   {
      messageFilter.set(null);
   }

   public void addMessageValidator(ControllerNetworkSubscriber.MessageValidator newValidator)
   {
      messageValidator.set(newValidator);
   }

   public void removeMessageValidator()
   {
      messageValidator.set(null);
   }

   @SuppressWarnings("unchecked")
   private <T extends Settable<T>> void createSubscribersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedControlMessages.size(); i++)
      { // Creating the subscribers
         Class<T> messageClass = (Class<T>) listOfSupportedControlMessages.get(i);
         T messageLocalInstance = ROS2TopicNameTools.newMessageInstance(messageClass);

         realtimeROS2Node.createSubscription(ControllerAPI.getTopic(baseTopic, messageClass), s ->
         {
            s.takeNextData(messageLocalInstance, null);
            receivedMessage(messageLocalInstance);
         });
      }
   }

   private <T extends Settable<T>> void createPublishersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         Class<T> messageClass = (Class<T>) listOfSupportedStatusMessages.get(i);
         statusMessagePublisherMap.put(messageClass, createPublisher(messageClass));
      }

      statusMessageOutputManager.attachGlobalStatusMessageListener(statusMessage -> publishStatusMessage(statusMessage));
   }

   private <T> void publishStatusMessage(T message)
   {
      ROS2Publisher<T> publisher = (ROS2Publisher<T>) statusMessagePublisherMap.get(message.getClass());
      publisher.publish(message);
   }

   private <T extends Settable<T>> ROS2Publisher<T> createPublisher(Class<T> messageClass)
   {
      return realtimeROS2Node.createPublisher(ControllerAPI.getTopic(baseTopic, messageClass));
   }

   @SuppressWarnings("unchecked")
   private <T extends Settable<T>> void receivedMessage(Settable<?> message)
   {
      if (DEBUG)
         LogTools.debug("Received message: {}, {}", message.getClass().getSimpleName(), message);

      if (messageValidator.get().validate(message) != null)
         return;

      if (testMessageWithMessageFilter(message))
         commandInputManager.submitMessage((T) message);
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
}
