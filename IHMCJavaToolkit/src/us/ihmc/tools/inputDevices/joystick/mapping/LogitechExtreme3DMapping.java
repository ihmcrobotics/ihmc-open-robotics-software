package us.ihmc.tools.inputDevices.joystick.mapping;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component;
import net.java.games.input.Event;
import net.java.games.input.Component.Identifier;

public enum LogitechExtreme3DMapping implements JoystickMapping
{
   TRIGGER,
   THROTTLE,
   HAT,
   STICK_ROLL,
   STICK_PITCH,
   STICK_YAW,
   BUTTON_2,
   BUTTON_3,
   BUTTON_4,
   BUTTON_5,
   BUTTON_6,
   BUTTON_7,
   BUTTON_8,
   BUTTON_9,
   BUTTON_10,
   BUTTON_11,
   BUTTON_12;

   public static final LogitechExtreme3DMapping[] values = values();

   private static final Map<Identifier, LogitechExtreme3DMapping> windowsIdentifierToMapping = new HashMap<>(values.length);
   private static final Map<Identifier, LogitechExtreme3DMapping> macIdentifierToMapping = new HashMap<>(values.length);
   private static final Map<Identifier, LogitechExtreme3DMapping> linuxIdentifierToMapping = new HashMap<>(values.length);

   private static final Map<LogitechExtreme3DMapping, Identifier> mappingToWindowsIdentifier = new HashMap<>(values.length);
   private static final Map<LogitechExtreme3DMapping, Identifier> mappingToMacIdentifier = new HashMap<>(values.length);
   private static final Map<LogitechExtreme3DMapping, Identifier> mappingToLinuxIdentifier = new HashMap<>(values.length);

   static
   {
      mapValues(TRIGGER, Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER);
      mapValues(THROTTLE, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER);
      mapValues(HAT, Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV);
      mapValues(STICK_ROLL, Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X);
      mapValues(STICK_PITCH, Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y);
      mapValues(STICK_YAW, Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ);
      mapValues(BUTTON_2, Identifier.Button._1, Identifier.Button._1, Identifier.Button.THUMB);
      mapValues(BUTTON_3, Identifier.Button._2, Identifier.Button._2, Identifier.Button.THUMB2);
      mapValues(BUTTON_4, Identifier.Button._3, Identifier.Button._3, Identifier.Button.TOP);
      mapValues(BUTTON_5, Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2);
      mapValues(BUTTON_6, Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE);
      mapValues(BUTTON_7, Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE);
      mapValues(BUTTON_8, Identifier.Button._7, Identifier.Button._7, Identifier.Button.BASE2);
      mapValues(BUTTON_9, Identifier.Button._8, Identifier.Button._8, Identifier.Button.BASE3);
      mapValues(BUTTON_10, Identifier.Button._9, Identifier.Button._9, Identifier.Button.BASE4);
      mapValues(BUTTON_11, Identifier.Button._10, Identifier.Button._10, Identifier.Button.BASE5);
      mapValues(BUTTON_12, Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6);
   }

   private static void mapValues(LogitechExtreme3DMapping mapping, Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
   {
      windowsIdentifierToMapping.put(windowsIdentifier, mapping);
      macIdentifierToMapping.put(macIdentifier, mapping);
      linuxIdentifierToMapping.put(linuxIdentifier, mapping);
      mappingToWindowsIdentifier.put(mapping, windowsIdentifier);
      mappingToMacIdentifier.put(mapping, macIdentifier);
      mappingToLinuxIdentifier.put(mapping, linuxIdentifier);
   }

   @Override
   public Identifier getIdentifier()
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         return mappingToWindowsIdentifier.get(this);
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         return mappingToMacIdentifier.get(this);
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         return mappingToLinuxIdentifier.get(this);
      }
      else
      {
         return null;
      }
   }
   
   public static LogitechExtreme3DMapping getMapping(Identifier identifier)
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         return windowsIdentifierToMapping.get(identifier);
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         return macIdentifierToMapping.get(identifier);
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         return linuxIdentifierToMapping.get(identifier);
      }
      else
      {
         return null;
      }
   }
   
   private static LogitechExtreme3DMapping getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static LogitechExtreme3DMapping getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
