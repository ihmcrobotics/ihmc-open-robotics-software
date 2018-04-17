package us.ihmc.pathPlanning.visibilityGraphs.ui.messager;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.API;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerSharedVariables;
import us.ihmc.robotEnvironmentAwareness.communication.REATopicListener;

public class SimpleUIMessager extends REAMessagerSharedVariables
{
   private final ConcurrentHashMap<Topic<?>, AtomicReference<Object>> javaFXThreadSyncedTopicListenerInputsMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<REATopicListener<Object>>> javaFXThreadSyncedTopicListenersMap = new ConcurrentHashMap<>();

   private final AnimationTimer animationTimer;

   public SimpleUIMessager(API messagerAPI)
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

   public <T> Property<T> createPropertyInput(Topic<T> topic)
   {
      return createPropertyInput(topic, null);
   }

   public <T> Property<T> createPropertyInput(Topic<T> topic, T defaultValue)
   {
      SimpleObjectProperty<T> input = new SimpleObjectProperty<T>(this, topic.getName(), defaultValue);
      bindPropertyToTopic(topic, input);
      return input;
   }

   public <M, P> void bindBidirectional(Topic<M> topic, Property<P> property, PropertyToMessageTypeConverter<M, P> converterToMessageType, boolean pushValue)
   {
      MessageBidirectionalBinding<M, P> bind = new MessageBidirectionalBinding<>(messageContent -> submitMessage(topic, messageContent), property,
                                                                                 converterToMessageType);
      property.addListener(bind);
      registerJavaFXSyncedTopicListener(topic, bind);
      if (pushValue)
         submitMessage(topic, converterToMessageType.convert(property.getValue()));
   }

   public <T> void bindBidirectional(Topic<T> topic, Property<T> property, boolean pushValue)
   {
      MessageBidirectionalBinding<T, T> bind = MessageBidirectionalBinding.createSingleTypedBinding(messageContent -> submitMessage(topic, messageContent),
                                                                                                    property);
      property.addListener(bind);
      registerJavaFXSyncedTopicListener(topic, bind);
      if (pushValue)
         submitMessage(topic, property.getValue());
   }

   public <T> void bindPropertyToTopic(Topic<T> topic, Property<T> propertyToBind)
   {
      registerJavaFXSyncedTopicListener(topic, propertyToBind::setValue);
   }

   public <T> void bindTopic(Topic<T> topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> submitMessage(topic, observableValue.getValue()));
   }

   @SuppressWarnings("unchecked")
   public <T> void registerJavaFXSyncedTopicListener(Topic<T> topic, REATopicListener<T> listener)
   {
      List<REATopicListener<Object>> topicListeners = javaFXThreadSyncedTopicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         AtomicReference<Object> listenerInput = (AtomicReference<Object>) createInput(topic);
         javaFXThreadSyncedTopicListenerInputsMap.put(topic, listenerInput);
         javaFXThreadSyncedTopicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((REATopicListener<Object>) listener);
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
