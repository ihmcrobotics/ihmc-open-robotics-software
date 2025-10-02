package org.ros.node.topic;

import java.util.concurrent.TimeUnit;
//import org.ros.internal.node.topic.TopicParticipant;

public interface Publisher<T> {
   void setLatchMode(boolean var1);

   boolean getLatchMode();

   T newMessage();

   void publish(T var1);

   boolean hasSubscribers();

   int getNumberOfSubscribers();

   void shutdown(long var1, TimeUnit var3);

   void shutdown();

   void addListener(PublisherListener<T> var1);
}