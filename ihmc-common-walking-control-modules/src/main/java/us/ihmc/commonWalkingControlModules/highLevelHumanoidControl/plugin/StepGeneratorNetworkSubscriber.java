package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;

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
   private final List<Class<? extends ROS2Message<?>>> listOfSupportedControlMessages;
   private final List<Class<? extends ROS2Message<?>>> listOfSupportedStatusMessages;

   private final Map<Class<? extends ROS2Message<?>>, ROS2Publisher<?>> statusMessagePublisherMap = new HashMap<>();

   private final AsyncROS2Node realtimeROS2Node;

   private final ROS2Topic<?> baseTopic;

   public StepGeneratorNetworkSubscriber(ROS2Topic<?> baseTopic,
                                         CommandInputManager csgCommandInputManager,
                                         StatusMessageOutputManager csgStatusMessageOutputManager,
                                         AsyncROS2Node realtimeROS2Node)
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

   public <T extends ROS2Message<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                             int expectedMessageSize,
                                                                             MessageUnpackingTools.MessageUnpacker<T> messageUnpacker)
   {
      registerSubcriberWithMessageUnpacker(multipleMessageType, baseTopic, expectedMessageSize, messageUnpacker);
   }

   public <T extends ROS2Message<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                             ROS2Topic<?> inputTopic,
                                                                             int expectedMessageSize,
                                                                             MessageUnpackingTools.MessageUnpacker<T> messageUnpacker)
   {
      registerSubcriberWithMessageUnpacker(multipleMessageType, inputTopic, null, expectedMessageSize, messageUnpacker);
   }

   public <T extends ROS2Message<T>> void registerSubcriberWithMessageUnpacker(Class<T> multipleMessageType,
                                                                             ROS2Topic<?> inputTopic,
                                                                             ROS2QoSProfile qosProfile,
                                                                             int expectedMessageSize,
                                                                             MessageUnpackingTools.MessageUnpacker<T> messageUnpacker)
   {
      final List<ROS2Message<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);

      ROS2Topic<T> topic = ControllerAPI.getTopic(inputTopic, multipleMessageType);
      T localInstance = ROS2Message.createInstance(multipleMessageType);

      if (qosProfile != null)
      {
         realtimeROS2Node.createSubscription(topic, reader ->
         {
            reader.read(localInstance);
            unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, localInstance);
         }, qosProfile);
      }
      else
      {
         realtimeROS2Node.createSubscription(topic, reader ->
         {
            reader.read(localInstance);
            unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, localInstance);
         });
      }
   }

   private <T extends ROS2Message<T>> void unpackMultiMessage(Class<T> multipleMessageHolderClass,
                                                           MessageUnpackingTools.MessageUnpacker<T> messageUnpacker,
                                                           List<ROS2Message<?>> unpackedMessages,
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
   private void createSubscribersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedControlMessages.size(); i++)
      { // Creating the subscribers
         Class<? extends ROS2Message<?>> messageClass = (Class<? extends ROS2Message<?>>) listOfSupportedControlMessages.get(i);
         @SuppressWarnings({"unchecked", "rawtypes"})
         Class messageClassRaw = messageClass;

         realtimeROS2Node.createSubscription(ControllerAPI.getTopic(baseTopic, messageClassRaw), reader -> receivedMessage(reader.read()));
      }
   }

   @SuppressWarnings("unchecked")
   private void createPublishersForSupportedMessages()
   {
      for (int i = 0; i < listOfSupportedStatusMessages.size(); i++)
      {
         Class<? extends ROS2Message<?>> messageClass = (Class<? extends ROS2Message<?>>) listOfSupportedStatusMessages.get(i);
         statusMessagePublisherMap.put(messageClass, createPublisher(messageClass));
      }

      statusMessageOutputManager.attachGlobalStatusMessageListener(statusMessage -> publishStatusMessage(statusMessage));
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private void publishStatusMessage(ROS2Message<?> message)
   {
      ROS2Publisher publisher = (ROS2Publisher) statusMessagePublisherMap.get(message.getClass());
      publisher.publish((ROS2Message) message);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private ROS2Publisher<?> createPublisher(Class<? extends ROS2Message<?>> messageClass)
   {
      return realtimeROS2Node.createPublisher(ControllerAPI.getTopic(baseTopic, (Class) messageClass));
   }

   @SuppressWarnings("unchecked")
   private <T extends ROS2Message<T>> void receivedMessage(ROS2Message<?> message)
   {
      if (DEBUG)
         LogTools.debug("Received message: {}, {}", message.getClass().getSimpleName(), message);

      if (messageValidator.get().validate(message) != null)
         return;

      if (testMessageWithMessageFilter(message))
         commandInputManager.submitMessage((T) message);
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
}
