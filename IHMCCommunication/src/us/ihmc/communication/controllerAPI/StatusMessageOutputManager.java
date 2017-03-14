package us.ihmc.communication.controllerAPI;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.concurrent.Builder;

/**
 * StatusMessageOutputManager is used to create an output API for a controller.
 * The controller can submit status messages through the method {@link #reportStatusMessage(StatusPacket)}.
 * Each status is then saved into a local copy.
 * This copy of the status to report is then given to all the corresponding listeners.
 * 
 * Listeners can subscribe to status messages using {@link #attachGlobalStatusMessageListener(GlobalStatusMessageListener)} and {@link #attachStatusMessageListener(Class, StatusMessageListener)}.
 * 
 * While a {@link StatusMessageListener} will only listen for a specific type of status message, a {@link GlobalStatusMessageListener} will get any status message reported by the controller.
 * 
 * The list of supported status messages can be accessed using {@link #getListOfSupportedMessages()}.
 *
 * @author Sylvain
 *
 */
public class StatusMessageOutputManager
{
   /** Local copies of the status messages reported by the controller. */
   private final Map<Class<? extends StatusPacket<?>>, StatusPacket<?>> statusClassToObjectMap = new HashMap<>();

   /** Exhaustive list of all the supported status messages that this API can process. */
   private final List<Class<? extends StatusPacket<?>>> listOfSupportedMessages = new ArrayList<>();

   /** Map of the listeners and the status message type they listen to. */
   private final Map<Class<? extends StatusPacket<?>>, List<StatusMessageListener<?>>> specificStatusMessageListenerMap = new HashMap<>();
   /** List of all the attached global listeners. */
   private final List<GlobalStatusMessageListener> globalStatusMessageListeners = new ArrayList<>();

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    * 
    * @param statusMessagesToRegister list of the status messages that this API should support.
    */
   public StatusMessageOutputManager(List<Class<? extends StatusPacket<?>>> statusMessagesToRegister)
   {
      registerStatusMessages(statusMessagesToRegister);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a list of status messages.
    * @param statusMessageClasses
    */
   @SuppressWarnings("unchecked")
   private <S extends StatusPacket<S>> void registerStatusMessages(List<Class<? extends StatusPacket<?>>> statusMessageClasses)
   {
      for (int i = 0; i < statusMessageClasses.size(); i++)
         registerStatusMessage((Class<S>) statusMessageClasses.get(i));
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a status message.
    * @param statusMessageClass
    */
   private <S extends StatusPacket<S>> void registerStatusMessage(Class<S> statusMessageClass)
   {
      Builder<S> builer = CommandInputManager.createBuilderWithEmptyConstructor(statusMessageClass);
      statusClassToObjectMap.put(statusMessageClass, builer.newInstance());
      listOfSupportedMessages.add(statusMessageClass);
   }

   /**
    * Attach a new global listener to this API.
    * @param globalStatusMessageListener the new global listener to attach.
    */
   public void attachGlobalStatusMessageListener(GlobalStatusMessageListener globalStatusMessageListener)
   {
      globalStatusMessageListeners.add(globalStatusMessageListener);
   }

   /**
    * Detach an existing global listener from this API.
    * @param globalStatusMessageListener the global listener to be removed.
    */
   public void detachGlobalStatusMessageListener(GlobalStatusMessageListener globalStatusMessageListener)
   {
      globalStatusMessageListeners.remove(globalStatusMessageListener);
   }

   /**
    * Attach a new listener to this API.
    * @param statusMessageClass refers to the type of status message the listener will be listening to.
    * @param statusMessageListener the listener to be attached.
    */
   public <S extends StatusPacket<S>> void attachStatusMessageListener(Class<S> statusMessageClass, StatusMessageListener<S> statusMessageListener)
   {
      if (statusClassToObjectMap.get(statusMessageClass) == null)
      {
         PrintTools.error(this, "The status message " + statusMessageClass.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }

      List<StatusMessageListener<?>> specificStatusMessageListenerList = specificStatusMessageListenerMap.get(statusMessageClass);
      if (specificStatusMessageListenerList == null)
      {
         specificStatusMessageListenerList = new ArrayList<>();
         specificStatusMessageListenerMap.put(statusMessageClass, specificStatusMessageListenerList);
      }

      specificStatusMessageListenerList.add(statusMessageListener);
   }

   /**
    * Detach a listener from this API.
    * @param statusMessageClass refers to the type of status message the listener was listening to.
    * @param statusMessageListener the listener to be removed.
    */
   public <S extends StatusPacket<S>> void detachStatusMessageListener(Class<S> statusMessageClass, StatusMessageListener<S> statusMessageListener)
   {
      List<StatusMessageListener<?>> specificStatusMessageListenerList = specificStatusMessageListenerMap.get(statusMessageClass);
      if (specificStatusMessageListenerList != null)
      {
         specificStatusMessageListenerList.remove(statusMessageListener);
      }
   }

   /**
    * Report a new status message that will be dispatched to all the listeners.
    * @param statusMessage the status message to report.
    */
   @SuppressWarnings("unchecked")
   public <S extends StatusPacket<S>> void reportStatusMessage(S statusMessage)
   {
      List<StatusMessageListener<?>> specificStatusMessageListeners = specificStatusMessageListenerMap.get(statusMessage.getClass());
      S statusMessageClone = (S) statusClassToObjectMap.get(statusMessage.getClass());

      if (statusMessageClone == null)
      {
         PrintTools.error(this, "The status message " + statusMessage.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }


      if (specificStatusMessageListeners != null)
      {
         for (int i = 0; i < specificStatusMessageListeners.size(); i++)
         {
            StatusMessageListener<S> statusMessageListener = (StatusMessageListener<S>) specificStatusMessageListeners.get(i);
            statusMessageClone.set(statusMessage);
            statusMessageListener.receivedNewMessageStatus(statusMessageClone);
         }
      }

      for (int i = 0; i < globalStatusMessageListeners.size(); i++)
      {
         GlobalStatusMessageListener globalStatusMessageListener = globalStatusMessageListeners.get(i);
         statusMessageClone.set(statusMessage);
         globalStatusMessageListener.receivedNewMessageStatus(statusMessageClone);
      }
   }

   /**
    * @return The list of all the status messages supported by this API. 
    */
   public List<Class<? extends StatusPacket<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   /**
    * Listener for a given type of status message.
    * The implementation of this interface will only receive status message of the given type.
    *
    * @param <S> The status message type to be listening for.
    */
   public static interface StatusMessageListener<S extends StatusPacket<S>>
   {
      public abstract void receivedNewMessageStatus(S statusMessage);
   }

   /**
    * Listener for all possible status message.
    * Implementations of this interface will receive any type of status message reported by the controller.
    */
   public static interface GlobalStatusMessageListener
   {
      public abstract void receivedNewMessageStatus(StatusPacket<?> statusMessage);
   }
}
