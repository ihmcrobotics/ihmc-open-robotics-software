package us.ihmc.behaviors.tools;

import java.util.HashSet;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.TopicListenerBase;
import us.ihmc.tools.thread.ActivationReference;

public class MessagerCallbackManager
{
   protected Messager messager;
   protected volatile boolean enabled = true;

   private final HashSet<Pair<Topic, TopicListenerBase>> topicListeners = new HashSet<>();
   private final HashSet<Pair<Topic, AtomicReference>> topicInputs = new HashSet<>();

   public void setMessager(Messager messager)
   {
      this.messager = messager;
      update();
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
      update();
   }

   private void update()
   {
      for (Pair<Topic, TopicListenerBase> listenerPair : topicListeners)
      {
         updateEnabledForListener(listenerPair);
      }

      for (Pair<Topic, AtomicReference> inputPair : topicInputs)
      {
         updateEnabledForInput(inputPair);
      }
   }

   private void updateEnabledForListener(Pair<Topic, TopicListenerBase> listenerPair)
   {
      if (messager != null)
      {
         if (enabled)
            messager.addTopicListenerBase(listenerPair.getLeft(), listenerPair.getRight());
         else
            messager.removeTopicListener(listenerPair.getLeft(), listenerPair.getRight());
      }
   }

   private void updateEnabledForInput(Pair<Topic, AtomicReference> inputPair)
   {
      if (messager != null)
      {
         if (enabled)
            messager.attachInput(inputPair.getLeft(), inputPair.getRight());
         else
            messager.removeInput(inputPair.getLeft(), inputPair.getRight());
      }
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic)
   {
      return new ActivationReference<>(createInput(topic, false), true);
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic)
   {
      return createInput(topic, null);
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic, T initialValue)
   {
      AtomicReference<T> input = new AtomicReference<>(initialValue);
      attachInput(topic, input);
      return input;
   }

   public <T> void attachInput(Topic<T> topic, AtomicReference<T> input)
   {
      Pair<Topic, AtomicReference> inputPair = Pair.of(topic, input);
      topicInputs.add(inputPair);
      updateEnabledForInput(inputPair);
   }

   public <T> boolean removeInput(Topic<T> topic, AtomicReference<T> input)
   {
      topicInputs.remove(Pair.of(topic, input));
      return messager == null || messager.removeInput(topic, input);
   }

   public <T> void addTopicListenerBase(Topic<T> topic, TopicListenerBase<T> listener)
   {
      Pair<Topic, TopicListenerBase> listenerPair = Pair.of(topic, listener);
      topicListeners.add(listenerPair);
      updateEnabledForListener(listenerPair);
   }

   public <T> boolean removeTopicListener(Topic<T> topic, TopicListenerBase<T> listener)
   {
      topicListeners.remove(Pair.of(topic, listener));
      return messager == null || messager.removeTopicListener(topic, listener);
   }

   public boolean isEnabled()
   {
      return enabled;
   }
}
