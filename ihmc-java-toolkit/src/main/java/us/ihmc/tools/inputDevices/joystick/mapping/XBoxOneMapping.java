package us.ihmc.tools.inputDevices.joystick.mapping;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.SystemUtils;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Event;
import us.ihmc.tools.inputDevices.joystick.JoystickCompatibilityFilter;

/**
 * OS X Mappings based on the following driver: https://github.com/360Controller/360Controller
 * using the default bindings/mappings
 */
public enum XBoxOneMapping implements JoystickMapping
{
   A,
   B,
   X,
   Y,
   LEFT_BUMPER,
   RIGHT_BUMPER,
   SELECT,
   START,
   XBOX_BUTTON,
   LEFT_TRIGGER,
   RIGHT_TRIGGER,
   LEFT_STICK_X,
   LEFT_STICK_Y,
   RIGHT_STICK_X,
   RIGHT_STICK_Y,
   LEFT_STICK_BUTTON,
   RIGHT_STICK_BUTTON,
   /** @deprecated Only works on Windows and Linux */
   DPAD,
   /** @deprecated Only works on Mac */
   DPAD_UP,
   /** @deprecated Only works on Mac */
   DPAD_DOWN,
   /** @deprecated Only works on Mac */
   DPAD_LEFT,
   /** @deprecated Only works on Mac */
   DPAD_RIGHT;

   public static final XBoxOneMapping[] values = values();

   private static final BiMap<Identifier, XBoxOneMapping> windowsBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, XBoxOneMapping> macBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, XBoxOneMapping> linuxBiMap = HashBiMap.create(values.length);

   static
   {
      mapValues(A, Identifier.Button._0, Identifier.Button._0, Identifier.Button.A);
      mapValues(B, Identifier.Button._1, Identifier.Button._1, Identifier.Button.B);
      mapValues(X, Identifier.Button._2, Identifier.Button._2, Identifier.Button.X);
      mapValues(Y, Identifier.Button._3, Identifier.Button._3, Identifier.Button.Y);
      mapValues(LEFT_BUMPER, Identifier.Button._4, Identifier.Button._4, Identifier.Button.LEFT_THUMB);
      mapValues(RIGHT_BUMPER, Identifier.Button._5, Identifier.Button._5, Identifier.Button.RIGHT_THUMB);
      mapValues(SELECT, Identifier.Button._6, Identifier.Button._9, Identifier.Button.SELECT);
      mapValues(START, Identifier.Button._7, Identifier.Button._8, Identifier.Button.START);
      mapValues(XBOX_BUTTON, Identifier.Button._10, Identifier.Button._10, Identifier.Button.MODE);
      mapValues(LEFT_TRIGGER, Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z);
      mapValues(RIGHT_TRIGGER, Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ);
      mapValues(LEFT_STICK_X, Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X);
      mapValues(LEFT_STICK_Y, Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y);
      mapValues(RIGHT_STICK_X, Identifier.Axis.RX, Identifier.Axis.RX, Identifier.Axis.RX);
      mapValues(RIGHT_STICK_Y, Identifier.Axis.RY, Identifier.Axis.RY, Identifier.Axis.RY);
      mapValues(LEFT_STICK_BUTTON, Identifier.Button._8, Identifier.Button._6, Identifier.Button.LEFT_THUMB3);
      mapValues(RIGHT_STICK_BUTTON, Identifier.Button._9, Identifier.Button._7, Identifier.Button.RIGHT_THUMB3);
      mapValues(DPAD, Identifier.Axis.POV, Identifier.Button.UNKNOWN, Identifier.Axis.POV);
      mapValues(DPAD_UP, Identifier.Button.UNKNOWN, Identifier.Button._11, Identifier.Button.UNKNOWN);
      mapValues(DPAD_DOWN, Identifier.Button.UNKNOWN, Identifier.Button._12, Identifier.Button.UNKNOWN);
      mapValues(DPAD_LEFT, Identifier.Button.UNKNOWN, Identifier.Button._13, Identifier.Button.UNKNOWN);
      mapValues(DPAD_RIGHT, Identifier.Button.UNKNOWN, Identifier.Button._14, Identifier.Button.UNKNOWN);
   }

   private static void mapValues(XBoxOneMapping mapping, Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
   {
      windowsBiMap.put(windowsIdentifier, mapping);
      macBiMap.put(macIdentifier, mapping);
      linuxBiMap.put(linuxIdentifier, mapping);
   }

   public static List<JoystickCompatibilityFilter> getCompatibilityFilters()
   {
      ArrayList<JoystickCompatibilityFilter> compatibilityFilters = new ArrayList<>();
      
      if (SystemUtils.IS_OS_LINUX)
      {
         compatibilityFilters.add(new JoystickCompatibilityFilter(LEFT_TRIGGER, false, 1.0, 0.5));
         compatibilityFilters.add(new JoystickCompatibilityFilter(RIGHT_TRIGGER, false, 1.0, 0.5));
      }
      
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
   
   public static XBoxOneMapping getMapping(Identifier identifier)
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
   
   private static XBoxOneMapping getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static XBoxOneMapping getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
