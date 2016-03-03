package us.ihmc.aware.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;

public class XboxControllerInputDevice implements InputDevice
{
   private static final String CONTROLLER_ID = "Microsoft X-Box One pad";
   private static final Map<Component.Identifier, InputDeviceAxis> AXES = new HashMap<>();

   static
   {
      AXES.put(Component.Identifier.Button.LEFT_THUMB, InputDeviceAxis.HOME_BUTTON);
      AXES.put(Component.Identifier.Button.SELECT, InputDeviceAxis.VIEW_BUTTON);
      // AXES.put(Component.Identifier.Button., MENU_BUTTON);
      AXES.put(Component.Identifier.Button.LEFT_THUMB, InputDeviceAxis.LEFT_BUTTON);
      AXES.put(Component.Identifier.Button.RIGHT_THUMB, InputDeviceAxis.RIGHT_BUTTON);
      AXES.put(Component.Identifier.Axis.Z, InputDeviceAxis.LEFT_TRIGGER);
      AXES.put(Component.Identifier.Axis.RZ, InputDeviceAxis.RIGHT_TRIGGER);
      AXES.put(Component.Identifier.Axis.X, InputDeviceAxis.LEFT_STICK_X);
      AXES.put(Component.Identifier.Axis.Y, InputDeviceAxis.LEFT_STICK_Y);
      AXES.put(Component.Identifier.Axis.RX, InputDeviceAxis.RIGHT_STICK_X);
      AXES.put(Component.Identifier.Axis.RY, InputDeviceAxis.RIGHT_STICK_Y);
      AXES.put(Component.Identifier.Button.A, InputDeviceAxis.BUTTON_A);
      AXES.put(Component.Identifier.Button.B, InputDeviceAxis.BUTTON_B);
      AXES.put(Component.Identifier.Button.X, InputDeviceAxis.BUTTON_X);
      AXES.put(Component.Identifier.Button.Y, InputDeviceAxis.BUTTON_Y);
      // AXES.put(Component.Identifier, D_PAD_UP);
      // AXES.put(Component.Identifier, D_PAD_RIGHT);
      // AXES.put(Component.Identifier, D_PAD_DOWN);
      // AXES.put(Component.Identifier, D_PAD_LEFT);
   }

   private final Controller controller;
   private final List<MotionEventCallback> callbacks = new ArrayList<>();

   public XboxControllerInputDevice()
   {
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

      for (Controller controller : controllers)
      {
         if (controller.getName().equals(CONTROLLER_ID))
         {
            this.controller = controller;
            return;
         }
      }

      // TODO: Throw a real exception
      throw new RuntimeException("No controller connected");
   }

   @Override
   public void registerCallback(MotionEventCallback callback)
   {
      callbacks.add(callback);
   }

   public void poll()
   {
      while (true)
      {
         controller.poll();

         Event event = new Event();
         while (controller.getEventQueue().getNextEvent(event))
         {
            InputDeviceAxis eventAxis = AXES.get(event.getComponent().getIdentifier());
            if (eventAxis != null)
            {
               MotionEvent motionEvent = new MotionEvent(eventAxis, event.getValue());
               notifyListeners(motionEvent);
            }
         }
      }
   }

   private void notifyListeners(MotionEvent event)
   {
      for (MotionEventCallback callback : callbacks)
      {
         callback.onMotionEvent(event);
      }
   }
}
