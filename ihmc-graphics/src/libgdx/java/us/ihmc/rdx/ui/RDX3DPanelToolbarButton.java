package us.ihmc.rdx.ui;

import us.ihmc.rdx.tools.RDXIconTexture;

public class RDX3DPanelToolbarButton
{
   private Runnable onPressed;
   private RDXIconTexture icon;
   private String tooltipText = null;

   public void setOnPressed(Runnable onPressed)
   {
      this.onPressed = onPressed;
   }

   public void setTooltipText(String text)
   {
      tooltipText = text;
   }

   public RDXIconTexture loadAndSetIcon(String iconAbsoluteResourcePath)
   {
      icon = new RDXIconTexture(iconAbsoluteResourcePath);
      return icon;
   }

   public void setIcon(RDXIconTexture iconTexture)
   {
      this.icon = iconTexture;
   }

   public void onPressed()
   {
      if (onPressed != null)
         onPressed.run();
   }

   public RDXIconTexture getIcon()
   {
      return icon;
   }

   public String getTooltipText()
   {
      return tooltipText;
   }
}