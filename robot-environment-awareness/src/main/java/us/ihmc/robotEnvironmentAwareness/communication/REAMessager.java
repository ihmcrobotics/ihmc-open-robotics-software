package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.API;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;

public interface REAMessager
{
   default void submitStateRequest(Topic<Boolean> requestTopic)
   {
      submitMessage(requestTopic, true);
   }

   default <T> void submitMessage(Topic<T> topic, T messageContent)
   {
      submitMessage(new REAMessage<T>(topic, messageContent));
   }

   <T> void submitMessage(REAMessage<T> message);

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

   API getMessagerAPI();
}