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

public enum Thrustmaster16000M implements JoystickMapping
{
   TRIGGER,
   THROTTLE,
   STICK_ROLL,
   STICK_PITCH,
   STICK_YAW,
   HAT,
   THUMB_BUTTON_CENTER,
   THUMB_BUTTON_LEFT,
   THUMB_BUTTON_RIGHT,
   BASE_BUTTON_LEFT_SIDE_UPPER_LEFT,
   BASE_BUTTON_LEFT_SIDE_UPPER_CENTER,
   BASE_BUTTON_LEFT_SIDE_UPPER_RIGHT,
   BASE_BUTTON_LEFT_SIDE_LOWER_LEFT,
   BASE_BUTTON_LEFT_SIDE_LOWER_CENTER,
   BASE_BUTTON_LEFT_SIDE_LOWER_RIGHT,
   /** @deprecated Only works on Windows and Mac */
   BASE_BUTTON_RIGHT_SIDE_UPPER_LEFT,
   BASE_BUTTON_RIGHT_SIDE_UPPER_CENTER,
   BASE_BUTTON_RIGHT_SIDE_UPPER_RIGHT,
   /** @deprecated Only works on Windows and Mac */
   BASE_BUTTON_RIGHT_SIDE_LOWER_LEFT,
   /** @deprecated Only works on Windows and Mac */
   BASE_BUTTON_RIGHT_SIDE_LOWER_CENTER,
   BASE_BUTTON_RIGHT_SIDE_LOWER_RIGHT,
   ;

   public static final Thrustmaster16000M[] values = values();

   private static final BiMap<Identifier, Thrustmaster16000M> windowsBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, Thrustmaster16000M> macBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, Thrustmaster16000M> linuxBiMap = HashBiMap.create(values.length);

   static
   {
      mapValues(TRIGGER, Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER);
      mapValues(THROTTLE, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER);
      mapValues(STICK_ROLL, Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X);
      mapValues(STICK_PITCH, Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y);
      mapValues(STICK_YAW, Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ);
      mapValues(HAT, Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV);
      mapValues(THUMB_BUTTON_CENTER, Identifier.Button._1, null, Identifier.Button.THUMB);
      mapValues(THUMB_BUTTON_LEFT, Identifier.Button._2, null, Identifier.Button.THUMB2);
      mapValues(THUMB_BUTTON_RIGHT, Identifier.Button._3, null, Identifier.Button.TOP);
      mapValues(BASE_BUTTON_LEFT_SIDE_UPPER_LEFT, Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2);
      mapValues(BASE_BUTTON_LEFT_SIDE_UPPER_CENTER, Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE);
      mapValues(BASE_BUTTON_LEFT_SIDE_UPPER_RIGHT, Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE);
      mapValues(BASE_BUTTON_LEFT_SIDE_LOWER_LEFT, Identifier.Button._9, Identifier.Button._9, Identifier.Button.BASE4);
      mapValues(BASE_BUTTON_LEFT_SIDE_LOWER_CENTER, Identifier.Button._8, Identifier.Button._8, Identifier.Button.BASE3);
      mapValues(BASE_BUTTON_LEFT_SIDE_LOWER_RIGHT, Identifier.Button._7, Identifier.Button._7, Identifier.Button.BASE2);
      mapValues(BASE_BUTTON_RIGHT_SIDE_UPPER_LEFT, Identifier.Button._12, Identifier.Button._12, Identifier.Button.UNKNOWN);
      mapValues(BASE_BUTTON_RIGHT_SIDE_UPPER_CENTER, Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6);
      mapValues(BASE_BUTTON_RIGHT_SIDE_UPPER_RIGHT, Identifier.Button._10, Identifier.Button._10, Identifier.Button.BASE5);
      mapValues(BASE_BUTTON_RIGHT_SIDE_LOWER_LEFT, Identifier.Button._13, Identifier.Button._13, Identifier.Button.UNKNOWN);
      mapValues(BASE_BUTTON_RIGHT_SIDE_LOWER_CENTER, Identifier.Button._14, Identifier.Button._14, Identifier.Button.UNKNOWN);
      mapValues(BASE_BUTTON_RIGHT_SIDE_LOWER_RIGHT, Identifier.Button._15, Identifier.Button._15, Identifier.Button.DEAD);
   }

   private static void mapValues(Thrustmaster16000M mapping, Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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
   
   public static Thrustmaster16000M getMapping(Identifier identifier)
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
   
   private static Thrustmaster16000M getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static Thrustmaster16000M getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
