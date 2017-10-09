package us.ihmc.tools.inputDevices.joystick.exceptions;

import java.io.IOException;

public class JoystickNotFoundException extends IOException
{
   private static final long serialVersionUID = -7645157801752756789L;

   public JoystickNotFoundException(String message)
   {
      super(message);
   }
}
