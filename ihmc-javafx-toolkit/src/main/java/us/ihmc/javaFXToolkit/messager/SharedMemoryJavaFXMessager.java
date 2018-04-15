package us.ihmc.javaFXToolkit.messager;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;

/**
 * Implementation of {@code JavaFXMessager} using shared memory.
 *
 * @author Sylvain Bertrand
 */
public class SharedMemoryJavaFXMessager extends SharedMemoryMessager implements JavaFXMessager
{
   private final ConcurrentHashMap<Topic<?>, AtomicReference<Object>> javaFXThreadSyncedTopicListenerInputsMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<TopicListener<Object>>> javaFXThreadSyncedTopicListenersMap = new ConcurrentHashMap<>();

   private final AnimationTimer animationTimer;

   /**
    * Creates a new messager.
    *
    * @param messagerAPI the API to use with this messager.
    */
   public SharedMemoryJavaFXMessager(MessagerAPI messagerAPI)
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

   /** {@inheritDoc} */
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

   /** {@inheritDoc} */
   @Override
   public void startMessager()
   {
      super.startMessager();
      animationTimer.start();
   }

   /** {@inheritDoc} */
   @Override
   public void closeMessager()
   {
      super.closeMessager();
      animationTimer.stop();
   }
}
