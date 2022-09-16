package us.ihmc.tools.gui;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;

public class YoAppearanceTools
{
   /**
    * To support making a transparent YoAppearance inline such as
    * YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4)
    */
   public static AppearanceDefinition makeTransparent(AppearanceDefinition appearance, double transparency)
   {
      appearance.setTransparency(transparency);
      return appearance;
   }
}
