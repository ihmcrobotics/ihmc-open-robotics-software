package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.concurrent.atomic.AtomicReference;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import us.ihmc.messager.Message;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerStateListener;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.MessageBidirectionalBinding;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;

// FIXME This implementation completely ignores that messages can be sent/received across several threads.
// TODO Review if the use of 2 messagers for REA is actually necessary.
public class REAUIMessager
{
   private final SharedMemoryJavaFXMessager internalMessager;
   private final Messager reaMessagerToModule;

   public REAUIMessager(Messager reaMessagerToModule)
   {
      this.reaMessagerToModule = reaMessagerToModule;
      internalMessager = new SharedMemoryJavaFXMessager(reaMessagerToModule.getMessagerAPI());
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic)
   {
      return createInput(topic, null);
   }

   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      AtomicReference<T> input = internalMessager.createInput(topic, defaultValue);
      reaMessagerToModule.addTopicListener(topic, input::set);
      return input;
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

   public <T> void broadcastMessage(Topic<T> topic, T messageContent)
   {
      submitMessageToModule(topic, messageContent);
      submitMessageInternal(topic, messageContent);
   }

   public <T> void broadcastMessage(Message<T> message)
   {
      submitMessageToModule(message);
      submitMessageInternal(message);
   }

   public void submitStateRequestToModule(Topic<Boolean> requestTopic)
   {
      reaMessagerToModule.submitMessage(requestTopic, true);
   }

   public <T> void submitMessageToModule(Topic<T> topic, T messageContent)
   {
      reaMessagerToModule.submitMessage(topic, messageContent);
   }

   public <T> void submitMessageToModule(Message<T> message)
   {
      reaMessagerToModule.submitMessage(message);
   }

   public <T> void submitMessageInternal(Topic<T> topic, T messageContent)
   {
      internalMessager.submitMessage(topic, messageContent);
   }

   public <T> void submitMessageInternal(Message<T> message)
   {
      internalMessager.submitMessage(message);
   }

   public <T> void registerTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      internalMessager.addTopicListener(topic, listener);
      reaMessagerToModule.addTopicListener(topic, listener);
   }

   public <M, P> void bindBidirectionalInternal(Topic<M> topic, Property<P> property, PropertyToMessageTypeConverter<M, P> converterToMessageType,
                                                boolean pushValue)
   {
      MessageBidirectionalBinding<M, P> bind = new MessageBidirectionalBinding<>(messageContent -> submitMessageInternal(topic, messageContent), property,
                                                                                 converterToMessageType);
      property.addListener(bind);
      internalMessager.addTopicListener(topic, bind);
      if (pushValue)
         internalMessager.submitMessage(topic, converterToMessageType.convert(property.getValue()));
   }

   public <T> void bindBidirectionalInternal(Topic<T> topic, Property<T> property, boolean pushValue)
   {
      MessageBidirectionalBinding<T, T> bind = MessageBidirectionalBinding.createSingleTypedBinding(messageContent -> submitMessageInternal(topic,
                                                                                                                                            messageContent),
                                                                                                    property);
      property.addListener(bind);
      internalMessager.addTopicListener(topic, bind);
      if (pushValue)
         internalMessager.submitMessage(topic, property.getValue());
   }

   public <T> void bindBidirectionalModule(Topic<T> topic, Property<T> property)
   {
      MessageBidirectionalBinding<T, T> bind = MessageBidirectionalBinding.createSingleTypedBinding(messageContent -> submitMessageToModule(topic,
                                                                                                                                            messageContent),
                                                                                                    property);
      property.addListener(bind);
      reaMessagerToModule.addTopicListener(topic, bind);
   }

   public <M, P> void bindBidirectionalGlobal(Topic<M> topic, Property<P> property, PropertyToMessageTypeConverter<M, P> converterToMessageType)
   {
      MessageBidirectionalBinding<M, P> bind = new MessageBidirectionalBinding<>(messageContent -> broadcastMessage(topic, messageContent), property,
                                                                                 converterToMessageType);
      property.addListener(bind);
      // TODO Workaround to prevent modification of the topic from another thread. Needs to be better implemented and propagated to the rest of this class.
      internalMessager.addFXTopicListener(topic, bind);
      reaMessagerToModule.addTopicListener(topic, message -> internalMessager.submitMessage(topic, message));
   }

   public <T> void bindBidirectionalGlobal(Topic<T> topic, Property<T> property)
   {
      MessageBidirectionalBinding<T, T> bind = MessageBidirectionalBinding.createSingleTypedBinding(messageContent -> broadcastMessage(topic, messageContent),
                                                                                                    property);
      property.addListener(bind);
      // TODO Workaround to prevent modification of the topic from another thread. Needs to be better implemented and propagated to the rest of this class.
      internalMessager.addFXTopicListener(topic, bind);
      reaMessagerToModule.addTopicListener(topic, message -> internalMessager.submitMessage(topic, message));
   }

   public <T> void bindPropertyToTopic(Topic<T> topic, Property<T> propertyToBind)
   {
      reaMessagerToModule.addTopicListener(topic, propertyToBind::setValue);
      internalMessager.addTopicListener(topic, propertyToBind::setValue);
   }

   public <T> void bindInternalTopic(Topic<T> topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> submitMessageInternal(topic, observableValue.getValue()));
   }

   public <T> void bindModuleTopic(Topic<T> topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> submitMessageToModule(topic, observableValue.getValue()));
   }

   public <T> void bindGlobal(Topic<T> topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> broadcastMessage(topic, observableValue.getValue()));
   }

   public void startMessager() throws Exception
   {
      internalMessager.startMessager();
      reaMessagerToModule.startMessager();
   }

   public void closeMessager() throws Exception
   {
      internalMessager.closeMessager();
      reaMessagerToModule.closeMessager();
   }

   public boolean isInternalMessagerOpen()
   {
      return internalMessager.isMessagerOpen();
   }

   public boolean isMessagerToModuleOpen()
   {
      return reaMessagerToModule.isMessagerOpen();
   }

   public void registerModuleMessagerStateListener(MessagerStateListener listener)
   {
      reaMessagerToModule.addMessagerStateListener(listener);
   }

   public void notifyModuleMessagerStateListeners()
   {
      reaMessagerToModule.notifyMessagerStateListeners();
   }
}
