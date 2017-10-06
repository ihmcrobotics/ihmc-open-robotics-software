package us.ihmc.tools.inputDevices.joystick.mapping;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.SystemUtils;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import us.ihmc.tools.inputDevices.joystick.JoystickCompatibilityFilter;
import net.java.games.input.Event;

public enum SaitekX52Mapping implements JoystickMapping
{
   /** Mappings for Windows, Mac, Linux */
   TRIGGER_PARITIAL,
   /** @deprecated Only works on Windows and Mac */
   TRIGGER_FULL,
   PINKY_TRIGGER,
   SAFE_FIRE,
   A,
   B,
   C,
   D,
   E,
   HAT_CENTER,
   /** Top left hat on stick */
   HAT2_UP,
   HAT2_RIGHT,
   HAT2_DOWN,
   HAT2_LEFT,
   /** Left index finger hat on throttle */
   HAT3_UP,
   HAT3_RIGHT,
   HAT3_DOWN,
   HAT3_LEFT,
   BASE_SWITCH_LEFT_UP,
   BASE_SWITCH_LEFT_DOWN,
   BASE_SWITCH_CENTER_UP,
   BASE_SWITCH_CENTER_DOWN,
   /** @deprecated Only works on Windows and Mac */
   BASE_SWITCH_RIGHT_UP,
   /** @deprecated Only works on Windows and Mac */
   BASE_SWITCH_RIGHT_DOWN,
   STICK_PITCH,
   STICK_ROLL,
   STICK_YAW,
   THROTTLE,
   LEFT_THUMB_SLIDER,
   BIG_ROTARY_KNOB,
   LITTLE_ROTARY_KNOB,
   /** @deprecated Only works on Windows and Mac */
   FUNCTION,
   /** @deprecated Only works on Linux */
   START_STOP,
   /** @deprecated Only works on Linux */
   RESET,
   /** @deprecated Only works on Linux */
   INFO,
   /** @deprecated Only works on Linux */
   MOUSE_TOGGLE,
   /** @deprecated Only works on Windows and Linux */
   NIPPLE_LEFT_RIGHT,
   /** @deprecated Only works on Windows and Linux */
   NIPPLE_UP_DOWN,
   ;
   
   public static final SaitekX52Mapping[] values = values();

   private static final BiMap<Identifier, SaitekX52Mapping> windowsBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, SaitekX52Mapping> macBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, SaitekX52Mapping> linuxBiMap = HashBiMap.create(values.length);

   static
   {
      mapValues(TRIGGER_PARITIAL, Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER);
      mapValues(TRIGGER_FULL, Identifier.Button._14, Identifier.Button._14, Identifier.Button.UNKNOWN);
      mapValues(PINKY_TRIGGER, Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE);
      mapValues(SAFE_FIRE, Identifier.Button._1, Identifier.Button._1, Identifier.Button.THUMB);
      mapValues(A, Identifier.Button._2, Identifier.Button._2, Identifier.Button.THUMB2);
      mapValues(B, Identifier.Button._3, Identifier.Button._3, Identifier.Button.TOP);
      mapValues(C, Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2);
      mapValues(D, Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE);
      mapValues(E, Identifier.Button._7, Identifier.Button._7, Identifier.Button.BASE2);
      mapValues(HAT_CENTER, Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV);
      mapValues(HAT2_UP, Identifier.Button._15, Identifier.Button._15, Identifier.Button.DEAD);
      mapValues(HAT2_RIGHT, Identifier.Button._16, Identifier.Button._16, Identifier.Button.EXTRA_1);
      mapValues(HAT2_DOWN, Identifier.Button._17, Identifier.Button._17, Identifier.Button.EXTRA_2);
      mapValues(HAT2_LEFT, Identifier.Button._18, Identifier.Button._18, Identifier.Button.EXTRA_3);
      mapValues(HAT3_UP, Identifier.Button._19, Identifier.Button._19, Identifier.Button.EXTRA_4);
      mapValues(HAT3_RIGHT, Identifier.Button._20, Identifier.Button._20, Identifier.Button.EXTRA_5);
      mapValues(HAT3_DOWN, Identifier.Button._21, Identifier.Button._21, Identifier.Button.EXTRA_6);
      mapValues(HAT3_LEFT, Identifier.Button._22, Identifier.Button._22, Identifier.Button.EXTRA_7);
      mapValues(BASE_SWITCH_LEFT_UP, Identifier.Button._8, Identifier.Button._8, Identifier.Button.BASE3);
      mapValues(BASE_SWITCH_LEFT_DOWN, Identifier.Button._9, Identifier.Button._9, Identifier.Button.BASE4);
      mapValues(BASE_SWITCH_CENTER_UP, Identifier.Button._10, Identifier.Button._10, Identifier.Button.BASE5);
      mapValues(BASE_SWITCH_CENTER_DOWN, Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6);
      mapValues(BASE_SWITCH_RIGHT_UP, Identifier.Button._12, Identifier.Button._12, Identifier.Button.UNKNOWN);
      mapValues(BASE_SWITCH_RIGHT_DOWN, Identifier.Button._13, Identifier.Button._13, Identifier.Button.UNKNOWN);
      mapValues(STICK_PITCH, Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y);
      mapValues(STICK_ROLL, Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X);
      mapValues(STICK_YAW, Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ);
      mapValues(THROTTLE, Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z);
      mapValues(LEFT_THUMB_SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER);
      mapValues(BIG_ROTARY_KNOB, Identifier.Axis.RY, Identifier.Axis.RY, Identifier.Axis.RY);
      mapValues(LITTLE_ROTARY_KNOB, Identifier.Axis.RX, Identifier.Axis.RX, Identifier.Axis.RX);
      mapValues(FUNCTION, null, null, Identifier.Button.EXTRA_11);
      mapValues(START_STOP, null, null, Identifier.Button.EXTRA_12);
      mapValues(RESET, null, null, Identifier.Button.EXTRA_13);
      mapValues(INFO, null, null, Identifier.Button.EXTRA_14);
      mapValues(MOUSE_TOGGLE, null, null, Identifier.Button.EXTRA_15);
      mapValues(NIPPLE_LEFT_RIGHT, Identifier.Axis.UNKNOWN, null, Identifier.Axis.SLIDER);
      mapValues(NIPPLE_UP_DOWN, Identifier.Axis.UNKNOWN, null, Identifier.Axis.SLIDER);
   }

   private static void mapValues(SaitekX52Mapping mapping, Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
   {
      windowsBiMap.put(windowsIdentifier, mapping);
      macBiMap.put(macIdentifier, mapping);
      linuxBiMap.put(linuxIdentifier, mapping);
   }

   public static List<JoystickCompatibilityFilter> getCompatibilityFilters()
   {
      ArrayList<JoystickCompatibilityFilter> compatibilityFilters = new ArrayList<>();
      
      return compatibilityFilters;
   }

   @Override
   public Identifier getIdentifier()
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         return windowsBiMap.inverse().get(this);
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         return macBiMap.inverse().get(this);
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         return linuxBiMap.inverse().get(this);
      }
      else
      {
         return null;
      }
   }
   
   public static SaitekX52Mapping getMapping(Identifier identifier)
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         return windowsBiMap.get(identifier);
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         return macBiMap.get(identifier);
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         return linuxBiMap.get(identifier);
      }
      else
      {
         return null;
      }
   }
   
   private static SaitekX52Mapping getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static SaitekX52Mapping getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
