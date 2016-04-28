package us.ihmc.quadrupedRobotics.input;

import java.util.ArrayList;
import java.util.List;

public abstract class PollingInputDevice
{
   /**
    * A list of callbacks to be notified when an event occurs.
    */
   private final List<InputEventCallback> callbacks = new ArrayList<>();

   /**
    * Block on the input device, notifying listeners as events are triggered.
    */
   public abstract void poll();

   public void registerCallback(InputEventCallback callback)
   {
      callbacks.add(callback);
   }

   protected void notifyListeners(InputEvent event)
   {
      for (InputEventCallback callback : callbacks)
      {
         callback.onInputEvent(event);
      }
   }
}
