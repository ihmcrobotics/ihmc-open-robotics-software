package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.tools.thread.ActivationReference;

import java.util.HashSet;
import java.util.concurrent.atomic.AtomicReference;

// TODO: This class should only implement a smaller subset of what a messager can do
public class ManagedMessager implements Messager
{
   private final Messager messager;
   private volatile boolean enabled = true;

   private final HashSet<Pair<Topic, TopicListener>> topicListeners = new HashSet<>();
   private final HashSet<Pair<Topic, AtomicReference>> topicInputs = new HashSet<>();

   public ManagedMessager(Messager messager)
   {
      this.messager = messager;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;

      for (Pair<Topic, TopicListener> listenerPair : topicListeners)
      {
         updateEnabledForListener(listenerPair);
      }

      for (Pair<Topic, AtomicReference> inputPair : topicInputs)
      {
         updateEnabledForInput(inputPair);
      }
   }

   private void updateEnabledForListener(Pair<Topic, TopicListener> listenerPair)
   {
      if (enabled)
         messager.registerTopicListener(listenerPair.getLeft(), listenerPair.getRight());
      else
         messager.removeTopicListener(listenerPair.getLeft(), listenerPair.getRight());
   }

   private void updateEnabledForInput(Pair<Topic, AtomicReference> inputPair)
   {
      if (enabled)
         messager.attachInput(inputPair.getLeft(), inputPair.getRight());
      else
         messager.removeInput(inputPair.getLeft(), inputPair.getRight());
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic)
   {
      return new ActivationReference<>(createInput(topic, false), true);
   }

   @Override
   public <T> void submitMessage(Topic<T> topic, T message)
   {
      if (enabled)
      {
         messager.submitMessage(topic, message);
      }
   }

   @Override
   public <T> void submitMessage(Message<T> message)
   {
      if (enabled)
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

   @Override
   public <T> void attachInput(Topic<T> topic, AtomicReference<T> input)
   {
      Pair<Topic, AtomicReference> inputPair = Pair.of(topic, input);
      topicInputs.add(inputPair);
      updateEnabledForInput(inputPair);
   }

   @Override
   public <T> boolean removeInput(Topic<T> topic, AtomicReference<T> input)
   {
      topicInputs.remove(Pair.of(topic, input));
      return messager.removeInput(topic, input);
   }

   @Override
   public <T> void registerTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      Pair<Topic, TopicListener> listenerPair = Pair.of(topic, listener);
      topicListeners.add(listenerPair);
      updateEnabledForListener(listenerPair);
   }

   @Override
   public <T> boolean removeTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      topicListeners.remove(Pair.of(topic, listener));
      return messager.removeTopicListener(topic, listener);
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
   public boolean isMessagerOpen()
   {
      return messager.isMessagerOpen();
   }

   @Override
   public void notifyMessagerStateListeners()
   {
      messager.notifyMessagerStateListeners();
   }

   @Override
   public void registerMessagerStateListener(MessagerStateListener listener)
   {
      messager.registerMessagerStateListener(listener);
   }

   @Override
   public boolean removeMessagerStateListener(MessagerStateListener listener)
   {
      return messager.removeMessagerStateListener(listener);
   }

   @Override
   public MessagerAPIFactory.MessagerAPI getMessagerAPI()
   {
      return messager.getMessagerAPI();
   }
}
