package us.ihmc.communication.controllerAPI;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.RequestPacket;
import us.ihmc.concurrent.Builder;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RequestMessageOutputManager
{
   /** Local copies of the requestable messages reported by the controller. */
   private final Map<Class<? extends RequestPacket<?>>, RequestPacket<?>> requestableClassToObjectMap = new HashMap<>();

   /** Exhaustive list of all the supported requestable messages that this API can process. */
   private final List<Class<? extends RequestPacket<?>>> listOfSupportedMessages = new ArrayList<>();
   /** List of all the attached global listeners. */
   private final List<GlobalRequestedMessageListener> globalRequestedMessageListeners = new ArrayList<>();

   public RequestMessageOutputManager(List<Class<? extends RequestPacket<?>>> requestableMessagesToRegister)
   {
      registerRequestableMessages(requestableMessagesToRegister);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a list of requestable messages.
    * @param requestableMessageClasses
    */
   @SuppressWarnings("unchecked")
   private <S extends RequestPacket<S>> void registerRequestableMessages(List<Class<? extends RequestPacket<?>>> requestableMessageClasses)
   {
      for (int i = 0; i < requestableMessageClasses.size(); i++)
         registerStatusMessage((Class<S>) requestableMessageClasses.get(i));
   }

   /**
    * Attach a new global listener to this API.
    * @param globalRequestableMessageListener the new global listener to attach.
    */
   public void attachGlobalRequestableMessageListener(GlobalRequestedMessageListener globalRequestableMessageListener)
   {
      globalRequestedMessageListeners.add(globalRequestableMessageListener);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a requestable message.
    * @param requestableMessageClass
    */
   private <S extends RequestPacket<S>> void registerStatusMessage(Class<S> requestableMessageClass)
   {
      Builder<S> builer = CommandInputManager.createBuilderWithEmptyConstructor(requestableMessageClass);
      requestableClassToObjectMap.put(requestableMessageClass, builer.newInstance());
      listOfSupportedMessages.add(requestableMessageClass);
   }

   /**
    * Report a new requested message that will be dispatched to all the listeners.
    * @param requestedMessage the requested message to report.
    */
   @SuppressWarnings("unchecked")
   public <S extends RequestPacket<S>> void reportStatusMessage(S requestedMessage)
   {
      S requestedMessageClone = (S) requestableClassToObjectMap.get(requestedMessage.getClass());

      if (requestedMessageClone == null)
      {
         PrintTools.error(this, "The status message " + requestedMessage.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }

      for (int i = 0; i < globalRequestedMessageListeners.size(); i++)
      {
         GlobalRequestedMessageListener globalStatusMessageListener = globalRequestedMessageListeners.get(i);
         requestedMessageClone.set(requestedMessage);
         globalStatusMessageListener.receivedNewMessageStatus(requestedMessageClone);
      }
   }

   /**
    * @return The list of all the requestable messages supported by this API.
    */
   public List<Class<? extends RequestPacket<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   /**
    * Listener for a given type of requested message.
    * The implementation of this interface will only receive requested message of the given type.
    *
    * @param <S> The status message type to be listening for.
    */
   public static interface StatusMessageListener<S extends RequestPacket<S>>
   {
      public abstract void receivedNewMessageStatus(S statusMessage);
   }

   /**
    * Listener for all possible requested message.
    * Implementations of this interface will receive any type of requested message reported by the controller.
    */
   public static interface GlobalRequestedMessageListener
   {
      public abstract void receivedNewMessageStatus(RequestPacket<?> statusMessage);
   }
}
