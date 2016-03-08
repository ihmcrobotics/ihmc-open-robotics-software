package us.ihmc.aware.input.devices;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import us.ihmc.aware.input.InputChannel;
import us.ihmc.aware.input.InputDevice;
import us.ihmc.aware.input.InputEvent;

public class XboxControllerInputDevice extends InputDevice
{
   private static final String CONTROLLER_ID = "Microsoft X-Box One pad";
   private static final Map<Component.Identifier, InputChannel> AXES = new HashMap<>();

   static
   {
      AXES.put(Component.Identifier.Button.MODE, InputChannel.HOME_BUTTON);
      AXES.put(Component.Identifier.Button.SELECT, InputChannel.VIEW_BUTTON);
//      AXES.put(Component.Identifier, InputChannel.MENU_BUTTON);
      AXES.put(Component.Identifier.Button.LEFT_THUMB, InputChannel.LEFT_BUTTON);
      AXES.put(Component.Identifier.Button.RIGHT_THUMB, InputChannel.RIGHT_BUTTON);
      AXES.put(Component.Identifier.Axis.Z, InputChannel.LEFT_TRIGGER);
      AXES.put(Component.Identifier.Axis.RZ, InputChannel.RIGHT_TRIGGER);
      AXES.put(Component.Identifier.Axis.X, InputChannel.LEFT_STICK_X);
      AXES.put(Component.Identifier.Axis.Y, InputChannel.LEFT_STICK_Y);
      AXES.put(Component.Identifier.Axis.RX, InputChannel.RIGHT_STICK_X);
      AXES.put(Component.Identifier.Axis.RY, InputChannel.RIGHT_STICK_Y);
      AXES.put(Component.Identifier.Button.A, InputChannel.BUTTON_A);
      AXES.put(Component.Identifier.Button.B, InputChannel.BUTTON_B);
      AXES.put(Component.Identifier.Button.X, InputChannel.BUTTON_X);
      AXES.put(Component.Identifier.Button.Y, InputChannel.BUTTON_Y);
      // AXES.put(Component.Identifier, D_PAD_UP);
      // AXES.put(Component.Identifier, D_PAD_RIGHT);
      // AXES.put(Component.Identifier, D_PAD_DOWN);
      // AXES.put(Component.Identifier, D_PAD_LEFT);
   }

   private final Controller controller;

   public XboxControllerInputDevice() throws IOException
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
      throw new IOException("No controller connected");
   }

   @Override
   public void poll()
   {
      while (true)
      {
         controller.poll();

         Event event = new Event();
         while (controller.getEventQueue().getNextEvent(event))
         {
            InputChannel eventAxis = AXES.get(event.getComponent().getIdentifier());
            if (eventAxis != null)
            {
               InputEvent inputEvent = new InputEvent(eventAxis, event.getValue());
               super.notifyListeners(inputEvent);
            }
         }
      }
   }

}
