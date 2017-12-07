package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.API;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerSharedVariables;

public class SimpleUIMessager extends REAMessagerSharedVariables
{
   public SimpleUIMessager(API messagerAPI)
   {
      super(messagerAPI);
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
      registerTopicListener(topic, bind);
      if (pushValue)
         submitMessage(topic, converterToMessageType.convert(property.getValue()));
   }

   public <T> void bindBidirectional(Topic<T> topic, Property<T> property, boolean pushValue)
   {
      MessageBidirectionalBinding<T, T> bind = MessageBidirectionalBinding.createSingleTypedBinding(messageContent -> submitMessage(topic, messageContent),
                                                                                                    property);
      property.addListener(bind);
      registerTopicListener(topic, bind);
      if (pushValue)
         submitMessage(topic, property.getValue());
   }

   public <T> void bindPropertyToTopic(Topic<T> topic, Property<T> propertyToBind)
   {
      registerTopicListener(topic, propertyToBind::setValue);
   }

   public <T> void bindTopic(Topic<T> topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> submitMessage(topic, observableValue.getValue()));
   }
}
