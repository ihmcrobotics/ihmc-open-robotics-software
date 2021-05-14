package us.ihmc.quadrupedCommunication.networkProcessing;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.concurrent.Builder;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * OutputManager is used to create an output API for a module.
 * The module can broadcast messages through the method {@link #reportMessage(Settable)}.
 * Each message is then saved into a local copy.
 * This copy of the message to report is then given to all the corresponding listeners.
 *
 * Listeners can subscribe to output messages using {@link #attachGlobalOutputMessageListener(GlobalOutputMessageListener)} and {@link #attachMessageListener(Class, OutputMessageListener)}.
 *
 * While a {@link OutputMessageListener} will only listen for a specific type of output message, a {@link GlobalOutputMessageListener} will get any output message reported by the module.
 *
 * The list of supported messages can be accessed using {@link #getListOfSupportedMessages()}.
 *
 * @author Robert
 *
 */
public class OutputManager
{
   /** Local copies of the messages reported by the controller. */
   private final Map<Class<? extends Settable<?>>, Settable<?>> outputClassToObjectMap = new HashMap<>();
   private final Map<Class<? extends Settable<?>>, ROS2Topic> outputClassToTopicMap = new HashMap<>();

   /** Exhaustive list of all the supported messages that this API can process. */
   private final List<Class<? extends Settable<?>>> listOfSupportedMessages = new ArrayList<>();

   /** Map of the listeners and the message type they listen to. */
   private final Map<Class<? extends Settable<?>>, List<OutputMessageListener<?>>> specificOutputMessageListenerMap = new HashMap<>();
   /** List of all the attached global listeners. */
   private final List<GlobalOutputMessageListener> globalOutputMessageListeners = new ArrayList<>();

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param outputMessagesToRegister list of the output messages that this API should support.
    */
   public OutputManager(Map<Class<? extends Settable<?>>, ROS2Topic> outputMessagesToRegister)
   {
      registerOutputMessages(outputMessagesToRegister);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a list of output messages.
    * @param outputMessageClasses
    */
   @SuppressWarnings("unchecked")
   private <S extends Settable<S>> void registerOutputMessages(Map<Class<? extends Settable<?>>, ROS2Topic> outputMessageClasses)
   {
      for (Class<? extends Settable<?>> outputMessageClass : outputMessageClasses.keySet())
      {
         registerOutputMessage((Class<S>) outputMessageClass, outputMessageClasses.get(outputMessageClass));
      }
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a output message.
    * @param outputMessageClass
    */
   public <S extends Settable<S>> void registerOutputMessage(Class<S> outputMessageClass, ROS2Topic topicName)
   {
      Builder<S> builder = CommandInputManager.createBuilderWithEmptyConstructor(outputMessageClass);
      outputClassToObjectMap.put(outputMessageClass, builder.newInstance());
      outputClassToTopicMap.put(outputMessageClass, topicName);
      listOfSupportedMessages.add(outputMessageClass);
   }

   /**
    * Attach a new global listener to this API.
    * @param globalOutputMessageListener the new global listener to attach.
    */
   public void attachGlobalOutputMessageListener(GlobalOutputMessageListener globalOutputMessageListener)
   {
      globalOutputMessageListeners.add(globalOutputMessageListener);
   }

   /**
    * Detach an existing global listener from this API.
    * @param globalOutputMessageListener the global listener to be removed.
    */
   public void detachGlobalOutputMessageListener(GlobalOutputMessageListener globalOutputMessageListener)
   {
      globalOutputMessageListeners.remove(globalOutputMessageListener);
   }

   /**
    * Attach a new listener to this API.
    * @param outputMessageClass refers to the type of output message the listener will be listening to.
    * @param outputMessageListener the listener to be attached.
    */
   public <S extends Settable<S>> void attachMessageListener(Class<S> outputMessageClass, OutputMessageListener<S> outputMessageListener)
   {
      if (outputClassToObjectMap.get(outputMessageClass) == null)
      {
         PrintTools.error(this, "The output message " + outputMessageClass.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }

      List<OutputMessageListener<?>> specificOutputMessageListenerList = specificOutputMessageListenerMap.get(outputMessageClass);
      if (specificOutputMessageListenerList == null)
      {
         specificOutputMessageListenerList = new ArrayList<>();
         specificOutputMessageListenerMap.put(outputMessageClass, specificOutputMessageListenerList);
      }

      specificOutputMessageListenerList.add(outputMessageListener);
   }

   /**
    * Detach a listener from this API.
    * @param outputMessageClass refers to the type of output message the listener was listening to.
    * @param outputMessageListener the listener to be removed.
    */
   public <S extends Settable<S>> void detachOutputMessageListener(Class<S> outputMessageClass, OutputMessageListener<S> outputMessageListener)
   {
      List<OutputMessageListener<?>> specificOutputMessageListenerList = specificOutputMessageListenerMap.get(outputMessageClass);
      if (specificOutputMessageListenerList != null)
      {
         specificOutputMessageListenerList.remove(outputMessageListener);
      }
   }

   /**
    * Report a new output message that will be dispatched to all the listeners.
    * @param outputMessage the output message to report.
    */
   @SuppressWarnings("unchecked")
   public <S extends Settable<S>> void reportMessage(S outputMessage)
   {
      List<OutputMessageListener<?>> specificOutputMessageListeners = specificOutputMessageListenerMap.get(outputMessage.getClass());
      S outputMessageClone = (S) outputClassToObjectMap.get(outputMessage.getClass());

      if (outputMessageClone == null)
      {
         PrintTools.error(this, "The output message " + outputMessage.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }


      if (specificOutputMessageListeners != null)
      {
         for (int i = 0; i < specificOutputMessageListeners.size(); i++)
         {
            OutputMessageListener<S> outputMessageListener = (OutputMessageListener<S>) specificOutputMessageListeners.get(i);
            outputMessageClone.set(outputMessage);
            outputMessageListener.receivedNewMessageOutput(outputMessageClone);
         }
      }

      for (int i = 0; i < globalOutputMessageListeners.size(); i++)
      {
         GlobalOutputMessageListener globalOutputMessageListener = globalOutputMessageListeners.get(i);
         outputMessageClone.set(outputMessage);
         globalOutputMessageListener.receivedNewMessageOutput(outputMessageClone);
      }
   }

   /**
    * @return The list of all the output messages supported by this API.
    */
   public List<Class<? extends Settable<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   public ROS2Topic getMessageTopicName(Class<? extends Settable<?>> messageClass)
   {
      return outputClassToTopicMap.get(messageClass);
   }

   /**
    * Listener for a given type of output message.
    * The implementation of this interface will only receive output message of the given type.
    *
    * @param <S> The output message type to be listening for.
    */
   public static interface OutputMessageListener<S extends Settable<S>>
   {
      void receivedNewMessageOutput(S outputMessage);
   }

   /**
    * Listener for all possible output message.
    * Implementations of this interface will receive any type of output message reported by the controller.
    */
   public static interface GlobalOutputMessageListener
   {
      void receivedNewMessageOutput(Settable<?> outputMessage);
   }
}
