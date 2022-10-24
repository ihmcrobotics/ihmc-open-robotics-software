package us.ihmc.gdx.ui;

import us.ihmc.gdx.tools.GDXIconTexture;

public class GDX3DPanelToolbarButton
{
   private Runnable onPressed;
   private GDXIconTexture icon;
   private String tooltipText = null;

   public void setOnPressed(Runnable onPressed)
   {
      this.onPressed = onPressed;
   }

   public void setTooltipText(String text)
   {
      tooltipText = text;
   }

   public GDXIconTexture loadAndSetIcon(String iconAbsoluteResourcePath)
   {
      icon = new GDXIconTexture(iconAbsoluteResourcePath);
      return icon;
   }

   public void setIcon(GDXIconTexture iconTexture)
   {
      this.icon = iconTexture;
   }

   public void onPressed()
   {
      if (onPressed != null)
         onPressed.run();
   }

   public GDXIconTexture getIcon()
   {
      return icon;
   }

   public String getTooltipText()
   {
      return tooltipText;
   }
}