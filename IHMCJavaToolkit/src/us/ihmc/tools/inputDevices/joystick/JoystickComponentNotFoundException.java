package us.ihmc.tools.inputDevices.joystick;

import net.java.games.input.Component.Identifier;

public class JoystickComponentNotFoundException extends RuntimeException
{
   private static final long serialVersionUID = 6467832335783181891L;

   public JoystickComponentNotFoundException(Identifier identifier)
   {
      super("Component with identifier " + identifier + " not found.");
   }
}
