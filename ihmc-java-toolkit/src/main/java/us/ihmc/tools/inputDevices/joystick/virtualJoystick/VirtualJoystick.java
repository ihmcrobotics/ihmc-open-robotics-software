package us.ihmc.tools.inputDevices.joystick.virtualJoystick;

import java.io.IOException;

import net.java.games.input.Component.Identifier;
import us.ihmc.tools.inputDevices.joystick.Joystick;

public class VirtualJoystick extends Joystick
{
   public VirtualJoystick(String name) throws IOException
   {
      super(new VirtualJoystickController(name));
   }
   
   public void queueEvent(Identifier identifier, double value)
   {
      ((VirtualJoystickController) joystickController).queueEvent(identifier, value);
   }
}
