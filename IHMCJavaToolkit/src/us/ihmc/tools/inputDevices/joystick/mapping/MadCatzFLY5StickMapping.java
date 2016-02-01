package us.ihmc.tools.inputDevices.joystick.mapping;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component.Identifier;

public enum MadCatzFLY5StickMapping
{
   TRIGGER(Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER),
   THROTTLE_LEFT(Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z),
   THROTTLE_RIGHT(Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER),
   /** @deprecated Not supported on Windows */
   THROTTLE_LOCK(null, Identifier.Button._1, Identifier.Button.THUMB),
   HAT(Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV),
   THUMB_SCOLL_UP(Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6),
   /** @deprecated Not identified on Linux */
   THUMB_SCOLL_DOWN(Identifier.Button._12, Identifier.Button._12, Identifier.Button.UNKNOWN),
   STICK_ROLL(Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X),
   STICK_PITCH(Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y),
   STICK_YAW(Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ),
   BUTTON_2(Identifier.Button._1, Identifier.Button._1, Identifier.Button.THUMB),
   BUTTON_3(Identifier.Button._2, Identifier.Button._2, Identifier.Button.THUMB2),
   BUTTON_4(Identifier.Button._3, Identifier.Button._3, Identifier.Button.TOP),
   BUTTON_5(Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2),
   BUTTON_6(Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE),
   BUTTON_7(Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE),
   BUTTON_8(Identifier.Button._7, Identifier.Button._7, Identifier.Button.BASE2),
   BUTTON_9(Identifier.Button._8, Identifier.Button._8, Identifier.Button.BASE3),
   BUTTON_10(Identifier.Button._9, Identifier.Button._9, Identifier.Button.BASE4),
   BUTTON_11(Identifier.Button._10, Identifier.Button._10, Identifier.Button.BASE5),
   /** @deprecated Not identified on Linux */
   BASE_MODE_BUTTON(Identifier.Button._10, Identifier.Button._10, Identifier.Button.UNKNOWN),
   ;
   
   public static final MadCatzFLY5StickMapping[] values = values();
   
   private final Identifier identifier;
   
   private MadCatzFLY5StickMapping(Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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
