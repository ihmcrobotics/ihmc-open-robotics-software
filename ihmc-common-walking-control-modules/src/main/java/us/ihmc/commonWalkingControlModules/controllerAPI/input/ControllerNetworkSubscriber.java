package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.apache.commons.lang3.StringUtils;

import com.google.common.base.CaseFormat;

import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.MessageCollectionNotification;
import controller_msgs.msg.dds.MessageCollectionPubSubType;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools.MessageUnpacker;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.RealtimeRos2Node;

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
   private static final boolean DEBUG = false;
   private static final String pubSubTypeGetterName = "getPubSubType";

   /** The input API to which the received messages should be submitted. */
   private final CommandInputManager controllerCommandInputManager;
   /** The output API that provides the status messages to send to the packet communicator. */
   private final StatusMessageOutputManager controllerStatusOutputManager;
   /** Used to filter messages coming in. */
   private final AtomicReference<MessageFilter> messageFilter;
   /** Used to filter messages coming in and report an error. */
   private final AtomicReference<MessageValidator> messageValidator;
   /** Used to synchronize the execution of a message collection. */
   private MessageCollector messageCollector = MessageCollector.createDummyCollector();

   /** All the possible status message that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedStatusMessages;

   /** All the possible messages that can be sent to the communicator. */
   private final List<Class<? extends Settable<?>>> listOfSupportedControlMessages;

   /**
    * Local buffers for each message to ensure proper copying from the controller thread to the
    * communication thread.
    */
   private final Map<Class<? extends Settable<?>>, IHMCRealtimeROS2Publisher<?>> statusMessagePublisherMap = new HashMap<>();

   private final RealtimeRos2Node realtimeRos2Node;

   public ControllerNetworkSubscriber(CommandInputManager controllerCommandInputManager, StatusMessageOutputManager controllerStatusOutputManager,
                                      RealtimeRos2Node realtimeRos2Node)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.controllerStatusOutputManager = controllerStatusOutputManager;
      this.realtimeRos2Node = realtimeRos2Node;
      listOfSupportedStatusMessages = controllerStatusOutputManager.getListOfSupportedMessages();
      listOfSupportedControlMessages = controllerCommandInputManager.getListOfSupportedMessages();

      messageFilter = new AtomicReference<>(message -> true);
      messageValidator = new AtomicReference<>(message -> null);

      if (realtimeRos2Node == null)
         PrintTools.error(this, "No ROS2 node, " + getClass().getSimpleName() + " cannot be created.");

      listOfSupportedStatusMessages.add(InvalidPacketNotificationPacket.class);

      createPublishersSubscribersForSupportedMessages();
      createGlobalStatusMessageListener();
   }

   private static <T> T newMessageInstance(Class<T> messageType)
   {
      try
      {
         return messageType.newInstance();
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         throw new RuntimeException("Something went wrong when invoking " + messageType.getSimpleName() + "'s empty constructor.", e);
      }
   }

   @SuppressWarnings({"unused", "unchecked"})
   private static <T> void getMessageTopicDataType(T messageInstance, Map<Class<?>, TopicDataType<?>> mapToModify)
   {
      Class<T> messageType = (Class<T>) messageInstance.getClass();

      if (mapToModify.containsKey(messageType))
         return;

      TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);

      mapToModify.put(messageType, topicDataType);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private static <T> TopicDataType<T> newMessageTopicDataTypeInstance(Class<T> messageType)
   {
      Method pubSubTypeGetter;

      try
      {
         pubSubTypeGetter = messageType.getDeclaredMethod(pubSubTypeGetterName);
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Something went wrong when looking up for the method " + messageType.getSimpleName() + "." + pubSubTypeGetterName + "().",
                                    e);
      }

      TopicDataType<T> topicDataType;

      try
      {
         topicDataType = (TopicDataType<T>) ((Supplier) pubSubTypeGetter.invoke(newMessageInstance(messageType))).get();
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when invoking the method " + messageType.getSimpleName() + "." + pubSubTypeGetterName + "().", e);
      }
      return topicDataType;
   }

   @SuppressWarnings("unchecked")
   public <T extends Settable<T>> void registerSubcriberWithMessageUnpacker(TopicDataType<T> multipleMessageTopicDataType, String topicName,
                                                                            int expectedMessageSize, MessageUnpacker<T> messageUnpacker)
   {
      Class<T> multipleMessageType = (Class<T>) multipleMessageTopicDataType.createData().getClass();
      final List<Settable<?>> unpackedMessages = new ArrayList<>(expectedMessageSize);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, multipleMessageTopicDataType, topicName,
                                           s -> unpackMultiMessage(multipleMessageType, messageUnpacker, unpackedMessages, s.takeNextData()));
   }

   @SuppressWarnings("unchecked")
   private <T extends Settable<T>> void unpackMultiMessage(Class<T> multipleMessageHolderClass, MessageUnpacker<T> messageUnpacker,
                                                           List<? extends Settable<?>> unpackedMessages, T multipleMessageHolder)
   {
      if (DEBUG)
         PrintTools.debug(ControllerNetworkSubscriber.this,
                          "Received message: " + multipleMessageHolder.getClass().getSimpleName() + ", " + multipleMessageHolder);

      String errorMessage = messageValidator.get().validate(multipleMessageHolder);

      if (errorMessage != null)
      {
         reportInvalidMessage(multipleMessageHolderClass, errorMessage);
         return;
      }

      if (testMessageWithMessageFilter(multipleMessageHolder))
      {
         messageUnpacker.unpackMessage(multipleMessageHolder, (List<Object>) unpackedMessages);

         for (int i = 0; i < unpackedMessages.size(); i++)
         {
            receivedMessage(unpackedMessages.get(i));
         }
         unpackedMessages.clear();
      }
   }

   public void addMessageCollector(MessageIDExtractor messageIDExtractor)
   {
      messageCollector = new MessageCollector(messageIDExtractor, listOfSupportedControlMessages);
      IHMCRealtimeROS2Publisher<MessageCollectionNotification> publisher = createPublisher(MessageCollectionNotification.class);
      listOfSupportedStatusMessages.add(MessageCollectionNotification.class);

      MessageCollection messageCollection = new MessageCollection();

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, new MessageCollectionPubSubType(), "/ihmc/message_collection", s -> {
         s.takeNextData(messageCollection, null);
         MessageCollectionNotification notification = messageCollector.startCollecting(messageCollection);
         publisher.publish(notification);
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
         T messageLocalInstance = newMessageInstance(messageClass);
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageClass);
         String topicName = createDefaultTopicName(messageClass);

         ROS2Tools.createCallbackSubscription(realtimeRos2Node, topicDataType, topicName, s -> {
            s.takeNextData(messageLocalInstance, null);
            receivedMessage(messageLocalInstance);
         });
      }
   }

   private <T extends Settable<T>> IHMCRealtimeROS2Publisher<T> createPublisher(Class<T> messageClass)
   {
      TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageClass);
      String topicName = createDefaultTopicName(messageClass);
      IHMCRealtimeROS2Publisher<T> publisher = ROS2Tools.createPublisher(realtimeRos2Node, topicDataType, topicName);
      return publisher;
   }

   private static <T extends Settable<T>> String createDefaultTopicName(Class<T> messageClass)
   {
      String topicName = messageClass.getSimpleName();
      topicName = StringUtils.removeEnd(topicName, "Packet");
      topicName = StringUtils.removeEnd(topicName, "Message");
      topicName = "/ihmc/" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, topicName);
      return topicName;
   }

   @SuppressWarnings("unchecked")
   private <T extends Settable<T>> void receivedMessage(Settable<?> message)
   {
      if (DEBUG)
         PrintTools.debug(ControllerNetworkSubscriber.this, "Received message: " + message.getClass().getSimpleName() + ", " + message);

      if (messageCollector.isCollecting() && messageCollector.interceptMessage(message))
      {
         if (DEBUG)
            PrintTools.debug(ControllerNetworkSubscriber.this, "Collecting message: " + message.getClass().getSimpleName() + ", " + message);

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
            PrintTools.error(ControllerNetworkSubscriber.this, "Packet failed to validate filter! Filter class: "
                  + messageFilter.get().getClass().getSimpleName() + ", rejected message: " + messageToTest.getClass().getSimpleName());
         return false;
      }
      return true;
   }

   private void reportInvalidMessage(Class<?> messageClass, String errorMessage)
   {
      publishStatusMessage(MessageTools.createInvalidPacketNotificationPacket(messageClass, errorMessage));
      PrintTools.error(ControllerNetworkSubscriber.this, "Packet failed to validate: " + messageClass.getSimpleName());
      PrintTools.error(ControllerNetworkSubscriber.this, errorMessage);
   }

   private void createGlobalStatusMessageListener()
   {
      controllerStatusOutputManager.attachGlobalStatusMessageListener(statusMessage -> publishStatusMessage(statusMessage));
   }

   @SuppressWarnings("unchecked")
   private <T> void publishStatusMessage(T message)
   {
      IHMCRealtimeROS2Publisher<T> publisher = (IHMCRealtimeROS2Publisher<T>) statusMessagePublisherMap.get(message.getClass());
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
