package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.concurrent.atomic.AtomicBoolean;

import javafx.beans.property.Property;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

public class MessageBidirectionalBinding<M, P> implements REATopicListener<M>, ChangeListener<P>
{
   private final AtomicBoolean changedOnMessageReception = new AtomicBoolean(false);
   private final Property<P> boundProperty;
   private final MessagingAction<M> messagingAction;
   private final PropertyToMessageTypeConverter<M, P> converter;

   public static <T> MessageBidirectionalBinding<T, T> createSingleTypedBinding(MessagingAction<T> messagingAction, Property<T> boundProperty)
   {
      return new MessageBidirectionalBinding<>(messagingAction, boundProperty, new OneToOneConverter<>());
   }

   public MessageBidirectionalBinding(MessagingAction<M> messagingAction, Property<P> boundProperty, PropertyToMessageTypeConverter<M, P> converter)
   {
      this.boundProperty = boundProperty;
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

   public interface MessagingAction<T>
   {
      public void doMessageAction(T messageContent);
   }

   /**
    * Allows ensure compatibility between a message and a property with different types.
    * For instance, when the property uses {@link Double} and the message is supposed to be an {@link Integer},
    * a converter from {@link Double} to {@link Integer} can be implemented allowing to bind the property to the message's topic.
    *
    * @param <C> Type of the message.
    * @param <P> Type of the property.
    */
   public interface PropertyToMessageTypeConverter<C, P>
   {
      /**
       * @param propertyValue the value coming from the property to be converted to the message type.
       * @return the corresponding value with the message's type.
       */
      public C convert(P propertyValue);
      /**
       * @param messageContent the message's value to be interpreted into the property's type.
       * @return the corresponding value with the property's type.
       */
      public P interpret(C messageContent);
   }

   private static class OneToOneConverter<T> implements PropertyToMessageTypeConverter<T, T>
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
