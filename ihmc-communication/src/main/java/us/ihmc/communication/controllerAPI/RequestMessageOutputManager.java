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
   /** Local copies of the request messages reported by the controller. */
   private final Map<Class<? extends RequestPacket<?>>, RequestPacket<?>> requestClassToObjectMap = new HashMap<>();

   /** Exhaustive list of all the supported request messages that this API can process. */
   private final List<Class<? extends RequestPacket<?>>> listOfSupportedMessages = new ArrayList<>();
   /** List of all the attached global listeners. */
   private final List<GlobalRequestedMessageListener> globalRequestMessageListeners = new ArrayList<>();

   public RequestMessageOutputManager(List<Class<? extends RequestPacket<?>>> requestMessagesToRegister)
   {
      registerRequestMessages(requestMessagesToRegister);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a list of request messages.
    * @param requestMessageClasses
    */
   @SuppressWarnings("unchecked")
   private <S extends RequestPacket<S>> void registerRequestMessages(List<Class<? extends RequestPacket<?>>> requestMessageClasses)
   {
      for (int i = 0; i < requestMessageClasses.size(); i++)
         registerStatusMessage((Class<S>) requestMessageClasses.get(i));
   }

   /**
    * Attach a new global listener to this API.
    * @param globalRequestMessageListener the new global listener to attach.
    */
   public void attachGlobalRequestMessageListener(GlobalRequestedMessageListener globalRequestMessageListener)
   {
      globalRequestMessageListeners.add(globalRequestMessageListener);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a request message.
    * @param requestMessageClass
    */
   private <S extends RequestPacket<S>> void registerStatusMessage(Class<S> requestMessageClass)
   {
      Builder<S> builer = CommandInputManager.createBuilderWithEmptyConstructor(requestMessageClass);
      requestClassToObjectMap.put(requestMessageClass, builer.newInstance());
      listOfSupportedMessages.add(requestMessageClass);
   }

   /**
    * Report a new requested message that will be dispatched to all the listeners.
    * @param requestedMessage the requested message to report.
    */
   @SuppressWarnings("unchecked")
   public <S extends RequestPacket<S>> void reportRequestMessage(S requestedMessage)
   {
      S requestedMessageClone = (S) requestClassToObjectMap.get(requestedMessage.getClass());

      if (requestedMessageClone == null)
      {
         PrintTools.error(this, "The request message " + requestedMessage.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }

      for (int i = 0; i < globalRequestMessageListeners.size(); i++)
      {
         GlobalRequestedMessageListener globalStatusMessageListener = globalRequestMessageListeners.get(i);
         requestedMessageClone.set(requestedMessage);
         globalStatusMessageListener.receivedNewMessageStatus(requestedMessageClone);
      }
   }

   /**
    * @return The list of all the request messages supported by this API.
    */
   public List<Class<? extends RequestPacket<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   /**
    * Listener for a given type of requested message.
    * The implementation of this interface will only receive requested message of the given type.
    *
    * @param <S> The request message type to be listening for.
    */
   public static interface RequestMessageListener<S extends RequestPacket<S>>
   {
      public abstract void receivedNewMessageRequeste(S requestMessage);
   }

   /**
    * Listener for all possible requested message.
    * Implementations of this interface will receive any type of requested message reported by the controller.
    */
   public static interface GlobalRequestedMessageListener
   {
      public abstract void receivedNewMessageStatus(RequestPacket<?> requestMessage);
   }
}
