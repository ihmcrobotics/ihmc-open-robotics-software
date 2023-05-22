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
   private RDXButtonIconTextures activeIconTextures;
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

   public RDXButtonIconTextures loadAndSetIcons(String iconAbsoluteResourcePath)
   {
      RDXIconTexture upIcon = new RDXIconTexture(iconAbsoluteResourcePath);
      String hoverPath = iconAbsoluteResourcePath.substring(0, iconAbsoluteResourcePath.indexOf(".png")) + "_hover.png";
      RDXIconTexture hoverIcon = new RDXIconTexture(hoverPath);
      String downPath = iconAbsoluteResourcePath.substring(0, iconAbsoluteResourcePath.indexOf(".png")) + "_down.png";
      RDXIconTexture downIcon = new RDXIconTexture(downPath);
      activeIconTextures = new RDXButtonIconTextures(upIcon, hoverIcon, downIcon);
      return activeIconTextures;
   }

   public void setActiveIconTextures(RDXButtonIconTextures activeIconTextures)
   {
      this.activeIconTextures = activeIconTextures;
   }

   public void onPressed()
   {
      if (onPressed != null)
         onPressed.run();
   }

   public RDXIconTexture getAppropriateIcon()
   {
      if (isDown)
         return activeIconTextures.getDownIcon();
      if (isHovered)
         return activeIconTextures.getHoverIcon();
      else
         return activeIconTextures.getUpIcon();
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