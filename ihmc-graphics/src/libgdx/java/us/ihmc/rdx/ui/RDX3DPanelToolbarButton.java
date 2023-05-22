package us.ihmc.rdx.ui;

import us.ihmc.rdx.tools.RDXIconTexture;

/**
 * Icons must be PNGs and should have transparent backgrounds.
 * We expect there to be an "up", "hover", and "down" icon.
 * Up is when the button is inactive, just sitting there.
 * Hover is when the user merely hovers over it with the mouse.
 * Down is when the mouse is hovered and the left mouse button
 * is in the down position.
 *
 * It's recommended that hover casts a drop shadow about 0.5 opacity
 * and down casts the same drop shadows at 0.8 opacity.
 */
public class RDX3DPanelToolbarButton
{
   private Runnable onPressed;
   private RDXIconTexture upIcon;
   private RDXIconTexture hoverIcon;
   private RDXIconTexture downIcon;
   private String tooltipText = null;
   private boolean isHovered = false;
   private boolean isDown = false;

   public void setOnPressed(Runnable onPressed)
   {
      this.onPressed = onPressed;
   }

   public void setTooltipText(String text)
   {
      tooltipText = text;
   }

   public void loadAndSetIcons(String iconAbsoluteResourcePath)
   {
      upIcon = new RDXIconTexture(iconAbsoluteResourcePath);
      String hoverPath = iconAbsoluteResourcePath.substring(0, iconAbsoluteResourcePath.indexOf(".png")) + "_hover.png";
      hoverIcon = new RDXIconTexture(hoverPath);
      String downPath = iconAbsoluteResourcePath.substring(0, iconAbsoluteResourcePath.indexOf(".png")) + "_down.png";
      downIcon = new RDXIconTexture(downPath);
   }

   public void setUpIcon(RDXIconTexture iconTexture)
   {
      this.upIcon = iconTexture;
   }

   public void onPressed()
   {
      if (onPressed != null)
         onPressed.run();
   }

   public RDXIconTexture getAppropriateIcon()
   {
      if (isDown)
         return downIcon;
      if (isHovered)
         return hoverIcon;
      else
         return upIcon;
   }

   public RDXIconTexture getUpIcon()
   {
      return upIcon;
   }

   public RDXIconTexture getHoverIcon()
   {
      return hoverIcon;
   }

   public RDXIconTexture getDownIcon()
   {
      return downIcon;
   }

   public String getTooltipText()
   {
      return tooltipText;
   }

   public boolean getHovered()
   {
      return isHovered;
   }

   public void setHovered(boolean hovered)
   {
      isHovered = hovered;
   }

   public boolean getDown()
   {
      return isDown;
   }

   public void setDown(boolean down)
   {
      isDown = down;
   }
}