package org.ros.node.topic;

import java.util.concurrent.TimeUnit;
import org.ros.message.MessageListener;

public interface Subscriber<T> {
   String TOPIC_MESSAGE_TYPE_WILDCARD = "*";

   void addMessageListener(MessageListener<T> var1, int var2);

   void addMessageListener(MessageListener<T> var1);

   void shutdown(long var1, TimeUnit var3);

   void shutdown();

   void addSubscriberListener(SubscriberListener<T> var1);

   boolean getLatchMode();
}