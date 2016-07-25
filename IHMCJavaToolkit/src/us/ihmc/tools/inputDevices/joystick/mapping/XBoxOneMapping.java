package us.ihmc.tools.inputDevices.joystick.mapping;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Event;

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

   private static final Map<Identifier, XBoxOneMapping> windowsIdentifierToMapping = new HashMap<>(values.length);
   private static final Map<Identifier, XBoxOneMapping> macIdentifierToMapping = new HashMap<>(values.length);
   private static final Map<Identifier, XBoxOneMapping> linuxIdentifierToMapping = new HashMap<>(values.length);

   private static final Map<XBoxOneMapping, Identifier> mappingToWindowsIdentifier = new HashMap<>(values.length);
   private static final Map<XBoxOneMapping, Identifier> mappingToMacIdentifier = new HashMap<>(values.length);
   private static final Map<XBoxOneMapping, Identifier> mappingToLinuxIdentifier = new HashMap<>(values.length);

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
   
   public static XBoxOneMapping getMapping(Identifier identifier)
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
   
   private static XBoxOneMapping getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static XBoxOneMapping getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
