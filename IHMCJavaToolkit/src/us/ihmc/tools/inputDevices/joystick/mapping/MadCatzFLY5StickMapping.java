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

public enum MadCatzFLY5StickMapping implements JoystickMapping
{
   TRIGGER,
   THROTTLE_LEFT,
   THROTTLE_RIGHT,
   /** @deprecated Not supported on Windows */
   THROTTLE_LOCK,
   HAT,
   THUMB_SCOLL_UP,
   /** @deprecated Not identified on Linux */
   THUMB_SCOLL_DOWN,
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
   /** @deprecated Not identified on Linux */
   BASE_MODE_BUTTON;

   public static final MadCatzFLY5StickMapping[] values = values();

   private static final BiMap<Identifier, MadCatzFLY5StickMapping> windowsBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, MadCatzFLY5StickMapping> macBiMap = HashBiMap.create(values.length);
   private static final BiMap<Identifier, MadCatzFLY5StickMapping> linuxBiMap = HashBiMap.create(values.length);

   static
   {
      mapValues(TRIGGER, Identifier.Button._0, Identifier.Button._0, Identifier.Button.TRIGGER);
      mapValues(THROTTLE_LEFT, Identifier.Axis.Z, Identifier.Axis.Z, Identifier.Axis.Z);
      mapValues(THROTTLE_RIGHT, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER, Identifier.Axis.SLIDER);
      mapValues(THROTTLE_LOCK, null, Identifier.Button._1, Identifier.Button.THUMB);
      mapValues(HAT, Identifier.Axis.POV, Identifier.Axis.POV, Identifier.Axis.POV);
      mapValues(THUMB_SCOLL_UP, Identifier.Button._11, Identifier.Button._11, Identifier.Button.BASE6);
      mapValues(THUMB_SCOLL_DOWN, Identifier.Button._12, Identifier.Button._12, Identifier.Button.UNKNOWN);
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
      mapValues(BASE_MODE_BUTTON, Identifier.Button._10, Identifier.Button._10, Identifier.Button.UNKNOWN);
   }

   private static void mapValues(MadCatzFLY5StickMapping mapping, Identifier windowsIdentifier, Identifier macIdentifier, Identifier linuxIdentifier)
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

   public static MadCatzFLY5StickMapping getMapping(Identifier identifier)
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
   
   private static MadCatzFLY5StickMapping getMapping(Component component)
   {
      return getMapping(component.getIdentifier());
   }

   public static MadCatzFLY5StickMapping getMapping(Event event)
   {
      return getMapping(event.getComponent());
   }
}
