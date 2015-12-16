package us.ihmc.tools.inputDevices.joystick;

import us.ihmc.tools.io.printing.PrintTools;

public enum JoystickModel
{
   LOGITECH_EXTREME_3D("Logitech Extreme 3D"),
   MAD_CATZ_V1_STICK("Mad Catz V.1 Stick"),
   MAD_CATZ_FLY5_STICK("Mad Catz F.L.Y.5 Stick"),
   SAITEK_X52("Saitek X52 Flight Control System"),
   UNKNOWN("Unknown"),
   
   ;
   
   public static final JoystickModel[] values = values();
   
   private final String jinputName;
   
   private JoystickModel(String jinputName)
   {
      this.jinputName = jinputName;
   }

   public String getJinputName()
   {
      return jinputName;
   }

   public static JoystickModel getModelFromName(String name)
   {
      for (JoystickModel joystickModel : values)
      {
         if (name.contains(joystickModel.getJinputName()))
            return joystickModel;
      }
      
      PrintTools.warn("Unknown joystick name: " + name);
      
      return UNKNOWN;
   }
}
