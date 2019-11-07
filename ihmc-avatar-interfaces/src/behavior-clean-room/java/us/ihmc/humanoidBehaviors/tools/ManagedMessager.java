package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.tuple.MutableTriple;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.tools.thread.ActivationReference;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

// TODO: This class should only implement a smaller subset of what a messager can do
public class ManagedMessager implements Messager
{
   private final Messager messager;
   private volatile boolean enabled = true;

   private final List<Pair<Topic, TopicListener>> topicListeners = new ArrayList<>();
   private final List<MutableTriple<Topic, AtomicReference, Object>> topicInputs = new ArrayList<>();

   public ManagedMessager(Messager messager)
   {
      this.messager = messager;
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
   public <T> AtomicReference<T> createInput(Topic<T> topic, T initialValue)
   {
      AtomicReference<T> input = messager.createInput(topic, initialValue);
      topicInputs.add(MutableTriple.of(topic, input, initialValue));
      return input;
   }

   @Override
   public <T> boolean removeInput(Topic<T> topic, AtomicReference<T> input)
   {
      return messager.removeInput(topic,input);
   }

   @Override
   public <T> void registerTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      topicListeners.add(Pair.of(topic, listener));
      messager.registerTopicListener(topic, listener);
   }

   @Override
   public <T> boolean removeTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      return removeTopicListener(topic, listener);
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
