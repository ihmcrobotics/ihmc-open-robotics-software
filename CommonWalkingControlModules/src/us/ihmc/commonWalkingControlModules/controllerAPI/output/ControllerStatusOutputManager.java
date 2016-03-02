package us.ihmc.commonWalkingControlModules.controllerAPI.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;

public class ControllerStatusOutputManager
{
   private final Map<Class<? extends StatusPacket<?>>, List<StatusMessageListener<?>>> specificStatusMessageListenerMap = new HashMap<>();
   private final List<GlobalStatusMessageListener> globalStatusMessageListeners = new ArrayList<>();

   public ControllerStatusOutputManager()
   {
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

   public <T extends StatusPacket<T>> void reportStatusMessage(T statusMessage)
   {
      List<StatusMessageListener<?>> specificStatusMessageListeners = specificStatusMessageListenerMap.get(statusMessage.getClass());

      if (specificStatusMessageListeners != null)
      {
         for (int i = 0; i < specificStatusMessageListeners.size(); i++)
         {
            @SuppressWarnings("unchecked")
            StatusMessageListener<T> statusMessageListener = (StatusMessageListener<T>) specificStatusMessageListeners.get(i);
            statusMessageListener.receivedNewMessageStatus(statusMessage);
         }
      }

      for (int i = 0; i < globalStatusMessageListeners.size(); i++)
      {
         GlobalStatusMessageListener globalStatusMessageListener = globalStatusMessageListeners.get(i);
         globalStatusMessageListener.receivedNewMessageStatus(statusMessage);
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

   public static interface StatusMessageListener<T extends StatusPacket<T>>
   {
      public abstract void receivedNewMessageStatus(T statusMessage);
   }

   public static interface GlobalStatusMessageListener
   {
      public abstract void receivedNewMessageStatus(StatusPacket<?> statusMessage);
   }
}
