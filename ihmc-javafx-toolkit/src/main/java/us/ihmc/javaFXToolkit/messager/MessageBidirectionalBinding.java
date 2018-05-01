package us.ihmc.javaFXToolkit.messager;

import java.util.concurrent.atomic.AtomicBoolean;

import javafx.beans.property.Property;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

/**
 * This class allows to create a bidirectional binding between a JavaFX {@link Property} to a
 * {@code Topic}.
 *
 * @author Sylvain Bertrand
 *
 * @param <M> the message data type.
 * @param <P> the property data type.
 */
public class MessageBidirectionalBinding<M, P> implements TopicListener<M>, ChangeListener<P>
{
   private final AtomicBoolean changedOnMessageReception = new AtomicBoolean(false);
   private final Property<P> boundProperty;
   private final MessagingAction<M> messagingAction;
   private final PropertyToMessageTypeConverter<M, P> converter;

   /**
    * Creates a binding with a property and a topic sharing the same data type.
    *
    * @param messagingAction the action to take when the property's value changes.
    * @param property the property to bind.
    * @return the binding.
    */
   public static <T> MessageBidirectionalBinding<T, T> createSingleTypedBinding(MessagingAction<T> messagingAction, Property<T> property)
   {
      return new MessageBidirectionalBinding<>(messagingAction, property, new DummyConverter<>());
   }

   /**
    * Creates a binding with a property and a topic with different data type.
    *
    * @param messagingAction the action to take when the property's value changes.
    * @param property the property to bind.
    * @param converter the protocol for converting data between the property and the topic.
    * @return the binding.
    */
   public MessageBidirectionalBinding(MessagingAction<M> messagingAction, Property<P> property, PropertyToMessageTypeConverter<M, P> converter)
   {
      this.boundProperty = property;
      this.messagingAction = messagingAction;
      this.converter = converter;
   }

   @Override
   public void changed(ObservableValue<? extends P> observable, P oldValue, P newValue)
   {
      if (changedOnMessageReception.getAndSet(false))
         return;
      M convertedPropertyValue = converter.convert(newValue);
      messagingAction.doMessageAction(convertedPropertyValue);
   }

   @Override
   public void receivedMessageForTopic(M messageContent)
   {
      P interpretedContent = converter.interpret(messageContent);
      changedOnMessageReception.set(!boundProperty.getValue().equals(interpretedContent));
      boundProperty.setValue(interpretedContent);
   }

   /**
    * Interface used to implement the protocol for updating the topic when the property's value is
    * changing.
    * 
    * @author Sylvain Bertrand
    *
    * @param <T> the topic data type.
    */
   public static interface MessagingAction<T>
   {
      public void doMessageAction(T messageContent);
   }

   /**
    * Allows to ensure compatibility between a message and a property with different types. For
    * instance, when the property uses {@link Double} and the message is supposed to be an
    * {@link Integer}, a converter from {@link Double} to {@link Integer} can be implemented
    * allowing to bind the property to the message's topic.
    *
    * @param <M> Type of the message.
    * @param <P> Type of the property.
    */
   public static interface PropertyToMessageTypeConverter<M, P>
   {
      /**
       * @param propertyValue the value coming from the property to be converted to the message
       *           type.
       * @return the corresponding value with the message's type.
       */
      public M convert(P propertyValue);

      /**
       * @param messageContent the message's value to be interpreted into the property's type.
       * @return the corresponding value with the property's type.
       */
      public P interpret(M messageContent);
   }

   private static class DummyConverter<T> implements PropertyToMessageTypeConverter<T, T>
   {
      @Override
      public T convert(T propertyValue)
      {
         return propertyValue;
      }

      @Override
      public T interpret(T messageContent)
      {
         return messageContent;
      }
   }
}
