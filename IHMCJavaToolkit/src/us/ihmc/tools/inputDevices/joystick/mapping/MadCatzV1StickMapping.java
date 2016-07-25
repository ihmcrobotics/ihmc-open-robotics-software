package us.ihmc.tools.inputDevices.joystick.mapping;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component;
import net.java.games.input.Event;
import net.java.games.input.Component.Identifier;

public enum MadCatzV1StickMapping implements JoystickMapping
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
   PINKY_TRIGGER;
   
   public static final MadCatzV1StickMapping[] values = values();

   private static final Map<Identifier, MadCatzV1StickMapping> windowsIdentifierToMapping = new HashMap<>(values.length);
   private static final Map<Identifier, MadCatzV1StickMapping> macIdentifierToMapping = new HashMap<>(values.length);
   private static final Map<Identifier, MadCatzV1StickMapping> linuxIdentifierToMapping = new HashMap<>(values.length);

   private static final Map<MadCatzV1StickMapping, Identifier> mappingToWindowsIdentifier = new HashMap<>(values.length);
   private static final Map<MadCatzV1StickMapping, Identifier> mappingToMacIdentifier = new HashMap<>(values.length);
   private static final Map<MadCatzV1StickMapping, Identifier> mappingToLinuxIdentifier = new HashMap<>(values.length);

   static
   {
      mapValues(TRIGGER, Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER);
      mapValues(THROTTLE, Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z);
      mapValues(HAT, Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV);
      mapValues(STICK_ROLL, Identifier.Axis.X, Identifier.Axis.X, Identifier.Axis.X);
      mapValues(STICK_PITCH, Identifier.Axis.Y, Identifier.Axis.Y, Identifier.Axis.Y);
      mapValues(STICK_YAW, Identifier.Axis.RZ, Identifier.Axis.RZ, Identifier.Axis.RZ);
      mapValues(BUTTON_2, Identifier.Button._1, Identifier.Button._1, Identifier.Button.THUMB);
      mapValues(BUTTON_3, Identifier.Button._2, Identifier.Button._2, Identifier.Button.THUMB2);
      mapValues(BUTTON_4, Identifier.Button._3, Identifier.Button._3, Identifier.Button.TOP);
      mapValues(BUTTON_5, Identifier.Button._4, Identifier.Button._4, Identifier.Button.TOP2);
      mapValues(BUTTON_6, Identifier.Button._5, Identifier.Button._5, Identifier.Button.PINKIE);
      mapValues(PINKY_TRIGGER, Identifier.Button._6, Identifier.Button._6, Identifier.Button.BASE);
   }

   private static void mapValues(MadCatzV1StickMapping mapping, Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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

   public static MadCatzV1StickMapping getMapping(Identifier identifier)
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
   
   private static MadCatzV1StickMapping getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static MadCatzV1StickMapping getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
