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
   private final Map<Class<? extends StatusPacket<?>>, List<StatusMessageListener<?>>> allStatusMessageListenerMap = new HashMap<>();

   public ControllerStatusOutputManager()
   {
   }

   public <T extends StatusPacket<T>> void attachStatusMessageListener(Class<T> statusMessageClass, StatusMessageListener<T> statusMessageListener)
   {
      List<StatusMessageListener<?>> statusMessageListenerList = allStatusMessageListenerMap.get(statusMessageClass);
      if (statusMessageListenerList == null)
      {
         statusMessageListenerList = new ArrayList<>();
         allStatusMessageListenerMap.put(statusMessageClass, statusMessageListenerList);
      }

      statusMessageListenerList.add(statusMessageListener);
   }

   public <T extends StatusPacket<T>> void detachStatusMessageListener(Class<T> statusMessageClass, StatusMessageListener<T> statusMessageListener)
   {
      List<StatusMessageListener<?>> statusMessageListenerList = allStatusMessageListenerMap.get(statusMessageClass);
      if (statusMessageListenerList != null)
      {
         statusMessageListenerList.remove(statusMessageListener);
      }
   }

   public <T extends StatusPacket<T>> void reportStatusMessage(T statusMessage)
   {
      List<StatusMessageListener<?>> statusMessageListeners = allStatusMessageListenerMap.get(statusMessage.getClass());

      if (statusMessageListeners == null)
         return;

      for (int i = 0; i < statusMessageListeners.size(); i++)
      {
         @SuppressWarnings("unchecked")
         StatusMessageListener<T> statusMessageListener = (StatusMessageListener<T>) statusMessageListeners.get(i);
         statusMessageListener.receivedNewMessageStatus(statusMessage);
      }
   }

   public void reportManipulationAbortedStatus()
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
}
