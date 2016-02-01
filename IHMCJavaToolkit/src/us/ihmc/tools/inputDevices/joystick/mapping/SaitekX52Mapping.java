package us.ihmc.tools.inputDevices.joystick.mapping;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component.Identifier;

public enum SaitekX52Mapping
{
   /** Mappings for Windows, Mac, Linux */
   TRIGGER_PARITIAL(Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER),
   /** @deprecated Only works on Windows and Mac */
   TRIGGER_FULL(Identifier.Button._14, Identifier.Button._14, Identifier.Button.UNKNOWN),
   PINKY_TRIGGER(Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE),
   SAFE_FIRE(Identifier.Button._1, Identifier.Button._1, Identifier.Button.THUMB),
   A(Identifier.Button._2, Identifier.Button._2, Identifier.Button.THUMB2),
   B(Identifier.Button._3, Identifier.Button._3, Identifier.Button.TOP),
   C(Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2),
   D(Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE),
   E(Identifier.Button._7, Identifier.Button._7, Identifier.Button.BASE2),
   HAT_CENTER(Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV),
   /** Top left hat on stick */
   HAT2_UP(Identifier.Button._15, Identifier.Button._15, Identifier.Button.DEAD),
   HAT2_RIGHT(Identifier.Button._16, Identifier.Button._16, Identifier.Button.EXTRA_1),
   HAT2_DOWN(Identifier.Button._17, Identifier.Button._17, Identifier.Button.EXTRA_2),
   HAT2_LEFT(Identifier.Button._18, Identifier.Button._18, Identifier.Button.EXTRA_3),
   /** Left index finger hat on throttle */
   HAT3_UP(Identifier.Button._19, Identifier.Button._19, Identifier.Button.EXTRA_4),
   HAT3_RIGHT(Identifier.Button._20, Identifier.Button._20, Identifier.Button.EXTRA_5),
   HAT3_DOWN(Identifier.Button._21, Identifier.Button._21, Identifier.Button.EXTRA_6),
   HAT3_LEFT(Identifier.Button._22, Identifier.Button._22, Identifier.Button.EXTRA_7),
   BASE_SWITCH_LEFT_UP(Identifier.Button._8, Identifier.Button._8, Identifier.Button.BASE3),
   BASE_SWITCH_LEFT_DOWN(Identifier.Button._9, Identifier.Button._9, Identifier.Button.BASE4),
   BASE_SWITCH_CENTER_UP(Identifier.Button._10, Identifier.Button._10, Identifier.Button.BASE5),
   BASE_SWITCH_CENTER_DOWN(Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6),
   /** @deprecated Only works on Windows and Mac */
   BASE_SWITCH_RIGHT_UP(Identifier.Button._12, Identifier.Button._12, Identifier.Button.UNKNOWN),
   /** @deprecated Only works on Windows and Mac */
   BASE_SWITCH_RIGHT_DOWN(Identifier.Button._13, Identifier.Button._13, Identifier.Button.UNKNOWN),
   STICK_PITCH(Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y),
   STICK_ROLL(Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X),
   STICK_YAW(Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ),
   THROTTLE(Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z),
   LEFT_THUMB_SLIDER(Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER),
   BIG_ROTARY_KNOB(Identifier.Axis.RY, Identifier.Axis.RY, Identifier.Axis.RY),
   LITTLE_ROTARY_KNOB(Identifier.Axis.RX, Identifier.Axis.RX, Identifier.Axis.RX),
   /** @deprecated Only works on Windows and Mac */
   FUNCTION(null, null, Identifier.Button.EXTRA_11),
   /** @deprecated Only works on Linux */
   START_STOP(null, null, Identifier.Button.EXTRA_12),
   /** @deprecated Only works on Linux */
   RESET(null, null, Identifier.Button.EXTRA_13),
   /** @deprecated Only works on Linux */
   INFO(null, null, Identifier.Button.EXTRA_14),
   /** @deprecated Only works on Linux */
   MOUSE_TOGGLE(null, null, Identifier.Button.EXTRA_15),
   /** @deprecated Only works on Windows and Linux */
   NIPPLE_LEFT_RIGHT(Identifier.Axis.UNKNOWN, null, Identifier.Axis.SLIDER),
   /** @deprecated Only works on Windows and Linux */
   NIPPLE_UP_DOWN(Identifier.Axis.UNKNOWN, null, Identifier.Axis.SLIDER),
   ;
   
   public static final SaitekX52Mapping[] values = values();
   
   private final Identifier identifier;
   
   private SaitekX52Mapping(Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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
