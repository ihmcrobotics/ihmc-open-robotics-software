package us.ihmc.tools.inputDevices.joystick.mapping;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component.Identifier;

public enum Thrustmaster16000M implements JoystickMapping
{
   TRIGGER(Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER),
   THROTTLE(Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER),
   STICK_ROLL(Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X),
   STICK_PITCH(Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y),
   STICK_YAW(Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ),
   HAT(Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV),
   THUMB_BUTTON_CENTER(Identifier.Button._1, null, Identifier.Button.THUMB),
   THUMB_BUTTON_LEFT(Identifier.Button._2, null, Identifier.Button.THUMB2),
   THUMB_BUTTON_RIGHT(Identifier.Button._3, null, Identifier.Button.TOP),
   BASE_BUTTON_LEFT_SIDE_UPPER_LEFT(Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2),
   BASE_BUTTON_LEFT_SIDE_UPPER_CENTER(Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE),
   BASE_BUTTON_LEFT_SIDE_UPPER_RIGHT(Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE),
   BASE_BUTTON_LEFT_SIDE_LOWER_LEFT(Identifier.Button._9, Identifier.Button._9, Identifier.Button.BASE4),
   BASE_BUTTON_LEFT_SIDE_LOWER_CENTER(Identifier.Button._8, Identifier.Button._8, Identifier.Button.BASE3),
   BASE_BUTTON_LEFT_SIDE_LOWER_RIGHT(Identifier.Button._7, Identifier.Button._7, Identifier.Button.BASE2),
   /** @deprecated Only works on Windows and Mac */
   BASE_BUTTON_RIGHT_SIDE_UPPER_LEFT(Identifier.Button._12, Identifier.Button._12, Identifier.Button.UNKNOWN),
   BASE_BUTTON_RIGHT_SIDE_UPPER_CENTER(Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6),
   BASE_BUTTON_RIGHT_SIDE_UPPER_RIGHT(Identifier.Button._10, Identifier.Button._10, Identifier.Button.BASE5),
   /** @deprecated Only works on Windows and Mac */
   BASE_BUTTON_RIGHT_SIDE_LOWER_LEFT(Identifier.Button._13, Identifier.Button._13, Identifier.Button.UNKNOWN),
   /** @deprecated Only works on Windows and Mac */
   BASE_BUTTON_RIGHT_SIDE_LOWER_CENTER(Identifier.Button._14, Identifier.Button._14, Identifier.Button.UNKNOWN),
   BASE_BUTTON_RIGHT_SIDE_LOWER_RIGHT(Identifier.Button._15, Identifier.Button._15, Identifier.Button.DEAD),
   ;

   public static final Thrustmaster16000M[] values = values();

   private final Identifier identifier;

   private Thrustmaster16000M(Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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
