package us.ihmc.tools.inputDevices.joystick;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.commons.PrintTools;

public enum JoystickModel
{
   LOGITECH_EXTREME_3D("Logitech Extreme 3D"),
   MAD_CATZ_V1_STICK("Mad Catz V.1 Stick"),
   MAD_CATZ_FLY5_STICK("Mad Catz F.L.Y.5 Stick"),
   SAITEK_X52("Saitek X52 Flight Control System"),
   THRUSTMASTER_16000M("T.16000M"),
   XBOX_ONE("Xbox One For Windows", "Xbox One Wired Controller", "Microsoft X-Box One pad"),
   UNKNOWN("Unknown"),
   
   ;
   
   public static final JoystickModel[] values = values();
   
   private final String jinputName;
   
   private JoystickModel(String jinputName)
   {
      this.jinputName = jinputName;
   }

   private JoystickModel(String jinputWindowsName, String jinputMacName, String jinputLinuxName)
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
}
