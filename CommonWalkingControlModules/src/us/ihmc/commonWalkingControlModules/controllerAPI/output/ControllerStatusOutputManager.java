package us.ihmc.commonWalkingControlModules.controllerAPI.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.CommandInputManager;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.concurrent.Builder;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.tools.io.printing.PrintTools;

public class ControllerStatusOutputManager
{
   private final Map<Class<? extends StatusPacket<?>>, StatusPacket<?>> statusClassToObjectMap = new HashMap<>();
   private final List<Class<? extends StatusPacket<?>>> listOfSupportedMessages;

   private final Map<Class<? extends StatusPacket<?>>, List<StatusMessageListener<?>>> specificStatusMessageListenerMap = new HashMap<>();
   private final List<GlobalStatusMessageListener> globalStatusMessageListeners = new ArrayList<>();

   public ControllerStatusOutputManager()
   {
      registerStatusMessage(CapturabilityBasedStatus.class);
      registerStatusMessage(FootstepStatus.class);
      registerStatusMessage(WalkingStatusMessage.class);
      registerStatusMessage(ManipulationAbortedStatus.class);

      listOfSupportedMessages = new ArrayList<>(statusClassToObjectMap.keySet());
   }

   private <T extends StatusPacket<T>> void registerStatusMessage(Class<T> statusMessageClass)
   {
      Builder<T> builer = CommandInputManager.createBuilderWithEmptyConstructor(statusMessageClass);
      statusClassToObjectMap.put(statusMessageClass, builer.newInstance());
   }

   public void attachGlobalStatusMessageListener(GlobalStatusMessageListener globalStatusMessageListener)
   {
      globalStatusMessageListeners.add(globalStatusMessageListener);
   }

   public void detachGlobalStatusMessageListener(GlobalStatusMessageListener globalStatusMessageListener)
   {
      globalStatusMessageListeners.remove(globalStatusMessageListener);
   }

   public <T extends StatusPacket<T>> void attachStatusMessageListener(Class<T> statusMessageClass, StatusMessageListener<T> statusMessageListener)
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

   public <T extends StatusPacket<T>> void detachStatusMessageListener(Class<T> statusMessageClass, StatusMessageListener<T> statusMessageListener)
   {
      List<StatusMessageListener<?>> specificStatusMessageListenerList = specificStatusMessageListenerMap.get(statusMessageClass);
      if (specificStatusMessageListenerList != null)
      {
         specificStatusMessageListenerList.remove(statusMessageListener);
      }
   }

   @SuppressWarnings("unchecked")
   public <T extends StatusPacket<T>> void reportStatusMessage(T statusMessage)
   {
      List<StatusMessageListener<?>> specificStatusMessageListeners = specificStatusMessageListenerMap.get(statusMessage.getClass());
      T statusMessageClone = (T) statusClassToObjectMap.get(statusMessage.getClass());

      if (statusMessageClone == null)
      {
         PrintTools.error(this, "The status message " + statusMessage.getClass().getSimpleName() + " has not been registered or is not supported.");
         return;
      }

      statusMessageClone.set(statusMessage);

      if (specificStatusMessageListeners != null)
      {
         for (int i = 0; i < specificStatusMessageListeners.size(); i++)
         {
            StatusMessageListener<T> statusMessageListener = (StatusMessageListener<T>) specificStatusMessageListeners.get(i);
            statusMessageListener.receivedNewMessageStatus(statusMessageClone);
         }
      }

      for (int i = 0; i < globalStatusMessageListeners.size(); i++)
      {
         GlobalStatusMessageListener globalStatusMessageListener = globalStatusMessageListeners.get(i);
         globalStatusMessageListener.receivedNewMessageStatus(statusMessageClone);
      }
   }

   public void reportManipulationAborted()
   {
      reportStatusMessage(new ManipulationAbortedStatus());
   }

   public void reportFootstepStatus(FootstepStatus statusMessage)
   {
      reportStatusMessage(statusMessage);
   }

   public void reportCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      reportStatusMessage(capturabilityBasedStatus);
   }

   public List<Class<? extends StatusPacket<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   public static interface StatusMessageListener<T extends StatusPacket<T>>
   {
      public abstract void receivedNewMessageStatus(T statusMessage);
   }

   public static interface GlobalStatusMessageListener
   {
      public abstract void receivedNewMessageStatus(StatusPacket<?> statusMessage);
   }
}
