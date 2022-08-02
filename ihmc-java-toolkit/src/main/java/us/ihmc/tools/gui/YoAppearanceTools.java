package us.ihmc.tools.gui;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;

public class YoAppearanceTools
{
   public static AppearanceDefinition makeTransparent(AppearanceDefinition appearance, double f)
   {
      appearance.setTransparency(f);
      return appearance;
   }
}
