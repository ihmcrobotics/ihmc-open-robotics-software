package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;

public interface REAMessager
{
   default void submitStateRequest(Topic<Boolean> requestTopic)
   {
      submitMessage(requestTopic, true);
   }

   default <T> void submitMessage(Topic<T> topic, T messageContent)
   {
      Message<T> reaMessage = new Message<>();
      reaMessage.topicID = topic.getUniqueID();
      reaMessage.messageContent = messageContent;
      submitMessage(reaMessage);
   }

   <T> void submitMessage(Message<T> message);

   <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue);

   default <T> AtomicReference<T> createInput(Topic<T> topic)
   {
      return createInput(topic, null);
   }

   <T> void registerTopicListener(Topic<T> topic, REATopicListener<T> listener);

   void startMessager() throws IOException;

   void closeMessager();

   boolean isMessagerOpen();

   void notifyConnectionStateListeners();

   void registerConnectionStateListener(ConnectionStateListener listener);

   MessagerAPI getMessagerAPI();
}