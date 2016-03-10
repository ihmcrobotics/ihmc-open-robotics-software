package us.ihmc.aware.input.devices;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import us.ihmc.aware.input.InputChannel;
import us.ihmc.aware.input.InputChannelConfig;
import us.ihmc.aware.input.InputDevice;
import us.ihmc.aware.input.InputEvent;

public class XboxControllerInputDevice extends InputDevice
{
   private static final String CONTROLLER_ID = "Microsoft X-Box One pad";
   private static final Map<Component.Identifier, InputChannelConfig> AXES = new HashMap<>();

   static
   {
      AXES.put(Component.Identifier.Button.MODE, new InputChannelConfig(InputChannel.HOME_BUTTON, false));
      AXES.put(Component.Identifier.Button.SELECT, new InputChannelConfig(InputChannel.VIEW_BUTTON, false));
      // AXES.put(Component.Identifier, new InputChannelConfig(InputChannel.MENU_BUTTON, false));
      AXES.put(Component.Identifier.Button.LEFT_THUMB, new InputChannelConfig(InputChannel.LEFT_BUTTON, false));
      AXES.put(Component.Identifier.Button.RIGHT_THUMB, new InputChannelConfig(InputChannel.RIGHT_BUTTON, false));
      AXES.put(Component.Identifier.Axis.Z, new InputChannelConfig(InputChannel.LEFT_TRIGGER, false));
      AXES.put(Component.Identifier.Axis.RZ, new InputChannelConfig(InputChannel.RIGHT_TRIGGER, false));
      AXES.put(Component.Identifier.Axis.X, new InputChannelConfig(InputChannel.LEFT_STICK_X, true));
      AXES.put(Component.Identifier.Axis.Y, new InputChannelConfig(InputChannel.LEFT_STICK_Y, true));
      AXES.put(Component.Identifier.Axis.RX, new InputChannelConfig(InputChannel.RIGHT_STICK_X, true));
      AXES.put(Component.Identifier.Axis.RY, new InputChannelConfig(InputChannel.RIGHT_STICK_Y, true));
      AXES.put(Component.Identifier.Button.A, new InputChannelConfig(InputChannel.BUTTON_A, false));
      AXES.put(Component.Identifier.Button.B, new InputChannelConfig(InputChannel.BUTTON_B, false));
      AXES.put(Component.Identifier.Button.X, new InputChannelConfig(InputChannel.BUTTON_X, false));
      AXES.put(Component.Identifier.Button.Y, new InputChannelConfig(InputChannel.BUTTON_Y, false));
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
            InputChannelConfig config = AXES.get(event.getComponent().getIdentifier());
            if (config != null)
            {
               double value = event.getValue();

               if (config.isInverted())
               {
                  value *= -1;
               }

               InputEvent inputEvent = new InputEvent(config.getChannel(), value);
               super.notifyListeners(inputEvent);
            }
         }
      }
   }

}
