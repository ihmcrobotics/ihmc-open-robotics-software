package us.ihmc.behaviors.tools.interfaces;

import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.TopicListener;
import us.ihmc.tools.thread.ActivationReference;

import java.util.concurrent.atomic.AtomicReference;

public interface MessagerPublishSubscribeAPI
{
   public <T> void publish(MessagerAPIFactory.Topic<T> topic, T message);

   public void publish(MessagerAPIFactory.Topic<Object> topic);

   public ActivationReference<Boolean> subscribeViaActivationReference(MessagerAPIFactory.Topic<Boolean> topic);

   public <T> void subscribeViaCallback(MessagerAPIFactory.Topic<T> topic, TopicListener<T> listener);

   public <T> AtomicReference<T> subscribeViaReference(MessagerAPIFactory.Topic<T> topic, T initialValue);

   public Notification subscribeTypelessViaNotification(MessagerAPIFactory.Topic<Object> topic);

   public void subscribeViaCallback(MessagerAPIFactory.Topic<Object> topic, Runnable callback);

   public <T extends K, K> TypedNotification<K> subscribeViaNotification(MessagerAPIFactory.Topic<T> topic);
}
