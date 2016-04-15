package us.ihmc.tools.inputDevices.joystick.mapping;

import net.java.games.input.Component.Identifier;
import org.apache.commons.lang3.SystemUtils;

public enum XBoxOneMapping implements JoystickMapping
{
   // TODO windows and mac mapping

   A(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.A),
   B(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.B),
   X(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.X),
   Y(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.Y),
   LEFT_BUMPER(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.LEFT_THUMB),
   RIGHT_BUMPER(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.RIGHT_THUMB),
   SELECT(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.SELECT),
   START(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.START),
   MODE(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.MODE),
   LEFT_TRIGGER(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Axis.Z),
   RIGHT_TRIGGER(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Axis.RZ),
   LEFT_JOYSTICK_X(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Axis.X),
   LEFT_JOYSTICK_Y(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Axis.Y),
   RIGHT_JOYSTICK_X(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Axis.RX),
   RIGHT_JOYSTICK_Y(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Axis.RY),
   LEFT_JOYSTICK_BUTTON(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.LEFT_THUMB3),
   RIGHT_JOYSTICK_BUTTON(Identifier.Button.UNKNOWN, Identifier.Button.UNKNOWN, Identifier.Button.RIGHT_THUMB3);
   
   private final Identifier identifier;

   private XBoxOneMapping(Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         identifier = windowsIdentifier;
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         identifier = macIdentifier;
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         identifier = linuxIdentifier;
      }
      else
      {
         identifier = null;
      }
   }

   @Override
   public Identifier getIdentifier()
   {
      return identifier;
   }

}
