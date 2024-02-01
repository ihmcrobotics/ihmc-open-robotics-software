package us.ihmc.behaviors.tools;

import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.Topic;

import java.util.concurrent.atomic.AtomicReference;

// TODO: This class should only implement a smaller subset of what a messager can do
public class ManagedMessager extends MessagerCallbackManager implements Messager
{
   public ManagedMessager(Messager messager)
   {
      this.messager = messager;
   }

   public ManagedMessager()
   {

   }

   @Override
   public <T> void submitMessage(Topic<T> topic, T message)
   {
      if (messager != null && enabled && isMessagerOpen())
      {
         messager.submitMessage(topic, message);
      }
   }

   @Override
   public <T> void submitMessage(Message<T> message)
   {
      if (messager != null && enabled && isMessagerOpen())
      {
         messager.submitMessage(message);
      }
   }

   @Override
   public <T> AtomicReference<T> createInput(Topic<T> topic)
   {
      return createInput(topic, null);
   }

   @Override
   public <T> AtomicReference<T> createInput(Topic<T> topic, T initialValue)
   {
      AtomicReference<T> input = new AtomicReference<>(initialValue);
      attachInput(topic, input);
      return input;
   }


   public boolean isEnabled()
   {
      return enabled;
   }

   @Override
   public boolean isMessagerOpen()
   {
      return messager.isMessagerOpen();
   }

   @Override
   public void addMessagerStateListener(MessagerStateListener listener)
   {
      messager.addMessagerStateListener(listener);
   }

   @Override
   public boolean removeMessagerStateListener(MessagerStateListener listener)
   {
      return messager.removeMessagerStateListener(listener);
   }

   @Override
   public void startMessager() throws Exception
   {
      messager.startMessager();
   }

   @Override
   public void closeMessager() throws Exception
   {
      messager.closeMessager();
   }

   @Override
   public void notifyMessagerStateListeners()
   {
      messager.notifyMessagerStateListeners();
   }

   @Override
   public MessagerAPIFactory.MessagerAPI getMessagerAPI()
   {
      return messager.getMessagerAPI();
   }
}
