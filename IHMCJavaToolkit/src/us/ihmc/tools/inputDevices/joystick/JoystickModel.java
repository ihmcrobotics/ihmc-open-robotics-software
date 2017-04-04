package us.ihmc.tools.inputDevices.joystick;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.commons.PrintTools;
import us.ihmc.tools.inputDevices.joystick.mapping.LogitechExtreme3DMapping;
import us.ihmc.tools.inputDevices.joystick.mapping.MadCatzFLY5StickMapping;
import us.ihmc.tools.inputDevices.joystick.mapping.MadCatzV1StickMapping;
import us.ihmc.tools.inputDevices.joystick.mapping.SaitekX52Mapping;
import us.ihmc.tools.inputDevices.joystick.mapping.Thrustmaster16000M;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public enum JoystickModel
{
   LOGITECH_EXTREME_3D("Logitech Extreme 3D", LogitechExtreme3DMapping.getCompatibilityFilters()),
   MAD_CATZ_V1_STICK("Mad Catz V.1 Stick", MadCatzV1StickMapping.getCompatibilityFilters()),
   MAD_CATZ_FLY5_STICK("Mad Catz F.L.Y.5 Stick", MadCatzFLY5StickMapping.getCompatibilityFilters()),
   SAITEK_X52("Saitek X52 Flight Control System", SaitekX52Mapping.getCompatibilityFilters()),
   THRUSTMASTER_16000M("T.16000M", Thrustmaster16000M.getCompatibilityFilters()),
   XBOX_ONE("Xbox One For Windows", "Xbox One Wired Controller", "Microsoft X-Box One pad", XBoxOneMapping.getCompatibilityFilters()),
   UNKNOWN("Unknown", new ArrayList<JoystickCompatibilityFilter>()),
   
   ;
   
   public static final JoystickModel[] values = values();
   
   private final String jinputName;
   private final List<JoystickCompatibilityFilter> compatibilityFilters;
   
   private JoystickModel(String jinputName, List<JoystickCompatibilityFilter> compatibilityFilters)
   {
      this.jinputName = jinputName;
      this.compatibilityFilters = compatibilityFilters;
   }

   private JoystickModel(String jinputWindowsName, String jinputMacName, String jinputLinuxName, List<JoystickCompatibilityFilter> compatibilityFilters)
   {
      if(SystemUtils.IS_OS_WINDOWS)
      {
         this.jinputName = jinputWindowsName;
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         this.jinputName = jinputMacName;
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         this.jinputName = jinputLinuxName;
      }
      else
      {
         this.jinputName = null;
      }
      
      this.compatibilityFilters = compatibilityFilters;
   }

   public String getJinputName()
   {
      return jinputName;
   }

   public static JoystickModel getModelFromName(String name)
   {
      for (JoystickModel joystickModel : values)
      {
         if (name.toLowerCase().contains(joystickModel.getJinputName().toLowerCase()))
            return joystickModel;
      }
      
      PrintTools.warn("Unknown joystick name: " + name);
      
      return UNKNOWN;
   }
   
   public List<JoystickCompatibilityFilter> getCompatibilityFilters()
   {
      return compatibilityFilters;
   }
}
