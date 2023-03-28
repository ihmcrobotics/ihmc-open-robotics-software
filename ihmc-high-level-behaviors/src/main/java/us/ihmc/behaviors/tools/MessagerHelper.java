package us.ihmc.behaviors.tools;

import us.ihmc.behaviors.tools.interfaces.MessagerPublishSubscribeAPI;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.tools.thread.ActivationReference;

import java.util.concurrent.atomic.AtomicReference;

/**
 * This class should support:
 * - Enabling and disabling callbacks
 * - Reconnecting
 * - Switching between Kryo and SharedMemory
 */
public class MessagerHelper implements MessagerPublishSubscribeAPI
{
   private final MessagerAPI messagerAPI;
   private boolean disconnecting = false;
   private Messager messager;
   private final ManagedMessager managedMessager = new ManagedMessager();

   public MessagerHelper(MessagerAPI messagerAPI)
   {
      this.messagerAPI = messagerAPI;
   }

   public void startServer(int port)
   {
      BehaviorMessagerUpdateThread updateThread = new BehaviorMessagerUpdateThread(getClass().getSimpleName(), 5);
      messager = KryoMessager.createServer(messagerAPI, port, updateThread);
      ThreadTools.startAThread(() -> ExceptionTools.handle(messager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION),
                               "KryoMessagerAsyncConnectionThread");
      managedMessager.setMessager(messager);
   }

   public void connectViaKryo(String hostname, int port)
   {
      BehaviorMessagerUpdateThread updateThread = new BehaviorMessagerUpdateThread(getClass().getSimpleName(), 5);
      messager = KryoMessager.createClient(messagerAPI, hostname, port, updateThread);
      ThreadTools.startAThread(() -> ExceptionTools.handle(messager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION),
                               "KryoMessagerAsyncConnectionThread");
      managedMessager.setMessager(messager);
   }

   public void connectViaSharedMemory(SharedMemoryMessager sharedMemoryMessager)
   {
      messager = sharedMemoryMessager;
      managedMessager.setMessager(messager);
   }

   public void setExternallyStartedMessager(Messager messager)
   {
      this.messager = messager;
      managedMessager.setMessager(messager);
   }

   public void disconnect()
   {
      if (messager instanceof SharedMemoryMessager)
      {
         messager = null;
      }
      else
      {
         disconnecting = true;
         ThreadTools.startAThread(() ->
         {
            if (messager != null)
            {
               ExceptionTools.handle(messager::closeMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION);
               messager = null;
            }
            disconnecting = false;
         }, "MessagerDisconnectionThread");
      }
   }

   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      managedMessager.setEnabled(enabled);
   }

   public boolean isDisconnecting()
   {
      return disconnecting;
   }

   public boolean isConnected()
   {
      return messager != null && !disconnecting && messager.isMessagerOpen();
   }

   public Messager getMessager()
   {
      return managedMessager;
   }

   public boolean isUsingSharedMemory()
   {
      return messager instanceof SharedMemoryMessager;
   }

   @Override
   public <T> void publish(MessagerAPIFactory.Topic<T> topic, T message)
   {
      managedMessager.submitMessage(topic, message);
   }

   @Override
   public void publish(MessagerAPIFactory.Topic<Object> topic)
   {
      managedMessager.submitMessage(topic, new Object());
   }

   @Override
   public ActivationReference<Boolean> subscribeViaActivationReference(MessagerAPIFactory.Topic<Boolean> topic)
   {
      return managedMessager.createBooleanActivationReference(topic);
   }

   @Override
   public <T> void subscribeViaCallback(MessagerAPIFactory.Topic<T> topic, TopicListener<T> listener)
   {
      managedMessager.addTopicListener(topic, listener);
   }

   @Override
   public <T> AtomicReference<T> subscribeViaReference(MessagerAPIFactory.Topic<T> topic, T initialValue)
   {
      return managedMessager.createInput(topic, initialValue);
   }

   @Override
   public Notification subscribeTypelessViaNotification(MessagerAPIFactory.Topic<Object> topic)
   {
      Notification notification = new Notification();
      subscribeViaCallback(topic, object -> notification.set());
      return notification;
   }

   @Override
   public void subscribeViaCallback(MessagerAPIFactory.Topic<Object> topic, Runnable callback)
   {
      subscribeViaCallback(topic, object -> callback.run());
   }

   @Override
   public <T extends K, K> TypedNotification<K> subscribeViaNotification(MessagerAPIFactory.Topic<T> topic)
   {
      TypedNotification<K> typedNotification = new TypedNotification<>();
      subscribeViaCallback(topic, typedNotification::set);
      return typedNotification;
   }
}
