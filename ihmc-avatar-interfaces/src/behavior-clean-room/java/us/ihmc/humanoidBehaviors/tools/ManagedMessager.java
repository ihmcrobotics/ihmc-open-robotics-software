package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.tuple.MutableTriple;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.tools.thread.ActivationReference;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ManagedMessager
{
   private final Messager messager;
   private volatile boolean enabled = true;

   private final List<Pair<Topic, TopicListener>> topicListeners = new ArrayList<>();
   private final List<MutableTriple<Topic, AtomicReference, Object>> topicInputs = new ArrayList<>();

   public ManagedMessager(Messager messager)
   {
      this.messager = messager;
   }

   public <T> void publish(Topic<T> topic, T message)
   {
      if (enabled)
      {
         messager.submitMessage(topic, message);
      }
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic)
   {
      return new ActivationReference<>(createInput(topic, false), true);
   }

   public <T> void registerCallback(Topic<T> topic, TopicListener<T> listener)
   {
      topicListeners.add(Pair.of(topic, listener));
      messager.registerTopicListener(topic, listener);
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic, T initialValue)
   {
      AtomicReference<T> input = messager.createInput(topic, initialValue);
      topicInputs.add(MutableTriple.of(topic, input, initialValue));
      return input;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;

      for (Pair<Topic, TopicListener> listenerPair : topicListeners)
      {
         if (enabled)
            messager.registerTopicListener(listenerPair.getLeft(), listenerPair.getRight());
         else
            messager.removeTopicListener(listenerPair.getLeft(), listenerPair.getRight());
      }
      // TODO: Fix required in ihmc-messager: https://github.com/ihmcrobotics/ihmc-messager/issues/6
      //      for (MutableTriple<Topic, AtomicReference, Object> inputTriple : topicInputs)
      //      {
      //         if (enabled)
      //            inputTriple.setMiddle(messager.createInput(inputTriple.getLeft(), inputTriple.getRight()));
      //         else
      //            messager.removeInput(inputTriple.getLeft(), inputTriple.getMiddle());
      //      }
   }
}
