package us.ihmc.pathPlanning.visibilityGraphs.ui.messager;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.SharedMemoryMessager;
import us.ihmc.javaFXToolkit.messager.TopicListener;

public class SimpleUIMessager extends SharedMemoryMessager implements JavaFXMessager
{
   private final ConcurrentHashMap<Topic<?>, AtomicReference<Object>> javaFXThreadSyncedTopicListenerInputsMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<TopicListener<Object>>> javaFXThreadSyncedTopicListenersMap = new ConcurrentHashMap<>();

   private final AnimationTimer animationTimer;

   public SimpleUIMessager(MessagerAPI messagerAPI)
   {
      super(messagerAPI);
      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            try
            {
               for (Topic<?> topic : javaFXThreadSyncedTopicListenersMap.keySet())
               {
                  Object listenersInput = javaFXThreadSyncedTopicListenerInputsMap.get(topic).getAndSet(null);
                  if (listenersInput != null)
                     javaFXThreadSyncedTopicListenersMap.get(topic).forEach(listener -> listener.receivedMessageForTopic(listenersInput));
               }
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      };
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> void registerJavaFXSyncedTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      List<TopicListener<Object>> topicListeners = javaFXThreadSyncedTopicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         AtomicReference<Object> listenerInput = (AtomicReference<Object>) createInput(topic);
         javaFXThreadSyncedTopicListenerInputsMap.put(topic, listenerInput);
         javaFXThreadSyncedTopicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((TopicListener<Object>) listener);
   }

   @Override
   public void startMessager() throws IOException
   {
      super.startMessager();
      animationTimer.start();
   }

   @Override
   public void closeMessager()
   {
      super.closeMessager();
      animationTimer.stop();
   }
}
