package us.ihmc.tools.inputDevices.joystick.mapping;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component.Identifier;

public enum MadCatzV1StickMapping
{
   TRIGGER(Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER),
   THROTTLE(Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z),
   HAT(Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV),
   STICK_ROLL(Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X),
   STICK_PITCH(Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y),
   STICK_YAW(Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ),
   BUTTON_2(Identifier.Button._1, Identifier.Button._1, Identifier.Button.THUMB),
   BUTTON_3(Identifier.Button._2, Identifier.Button._2, Identifier.Button.THUMB2),
   BUTTON_4(Identifier.Button._3, Identifier.Button._3, Identifier.Button.TOP),
   BUTTON_5(Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2),
   BUTTON_6(Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE),
   PINKY_TRIGGER(Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE),
   ;
   
   public static final MadCatzV1StickMapping[] values = values();
   
   private final Identifier identifier;
   
   private MadCatzV1StickMapping(Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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

   public Identifier getIdentifier()
   {
      return identifier;
   }
}
