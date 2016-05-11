package us.ihmc.tools.gui;

import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.plaf.metal.MetalLookAndFeel;
import javax.swing.plaf.nimbus.NimbusLookAndFeel;

public class SwingTools
{
   public enum LookAndFeelType
   {
      SYSTEM,
      CROSS_PLATFORM,
      METAL,
      NIMBUS,
   }
   
   public static void setLookAndFeel(LookAndFeelType type)
   {
      try
      {
         if (type == LookAndFeelType.SYSTEM)
         {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
         }
         else if (type == LookAndFeelType.CROSS_PLATFORM)
         {
            UIManager.setLookAndFeel(UIManager.getCrossPlatformLookAndFeelClassName());
         }
         else if (type == LookAndFeelType.METAL)
         {
            UIManager.setLookAndFeel(MetalLookAndFeel.class.getName());
         }
         else if (type == LookAndFeelType.NIMBUS)
         {
            UIManager.setLookAndFeel(NimbusLookAndFeel.class.getName());
         }
      }
      catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException e)
      {
         e.printStackTrace();
      }
   }
}
