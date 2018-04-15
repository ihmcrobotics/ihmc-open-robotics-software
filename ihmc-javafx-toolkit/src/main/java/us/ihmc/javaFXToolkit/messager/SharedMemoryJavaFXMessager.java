package us.ihmc.javaFXToolkit.messager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
   private final Map<Topic<?>, JavaFXSyncedTopicListeners> javaFXSyncedTopicListeners = new HashMap<>();
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
               for (JavaFXSyncedTopicListeners listener : javaFXSyncedTopicListeners.values())
                  listener.notifyListeners();
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
   public <T> void registerJavaFXSyncedTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      JavaFXSyncedTopicListeners topicListeners = javaFXSyncedTopicListeners.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new JavaFXSyncedTopicListeners(topic);
         javaFXSyncedTopicListeners.put(topic, topicListeners);
      }
      topicListeners.addListener(listener);
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

   @SuppressWarnings("unchecked")
   private class JavaFXSyncedTopicListeners
   {
      private final AtomicReference<Object> input;
      private final List<TopicListener<Object>> listeners = new ArrayList<>();

      private JavaFXSyncedTopicListeners(Topic<?> topic)
      {
         input = (AtomicReference<Object>) createInput(topic);
      }

      private void addListener(TopicListener<?> listener)
      {
         listeners.add((TopicListener<Object>) listener);
      }

      private void notifyListeners()
      {
         Object newData = input.getAndSet(null);
         if (newData != null)
            listeners.forEach(listener -> listener.receivedMessageForTopic(newData));
      }
   }
}
