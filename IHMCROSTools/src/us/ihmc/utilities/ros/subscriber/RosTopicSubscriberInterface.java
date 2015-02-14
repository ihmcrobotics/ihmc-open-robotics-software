package us.ihmc.utilities.ros.subscriber;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

public interface RosTopicSubscriberInterface<T> extends MessageListener<T>
{

   public abstract String getMessageType();

   public abstract void connected();
   
   public abstract void registered(Subscriber<T> subscriber);

   public void wailTillRegistered();

}