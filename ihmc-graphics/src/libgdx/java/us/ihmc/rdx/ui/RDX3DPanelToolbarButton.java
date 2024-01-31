package us.ihmc.rdx.ui;

import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.tools.Timer;

/**
 * Icons must be PNGs and should have transparent backgrounds.
 * We expect there to be an "up", "hover", and "down" icon.
 *
 * We want to place this icons in one PNG image with the "up"
 * in the left third of the image, the "hover" in the middle,
 * and the "down" on the right. We will use UV coordinates
 * to switch between them.
 *
 * "Up" is when the button is inactive, just sitting there.
 * "Hover" is when the user merely hovers over it with the mouse.
 * "Down" is when the mouse is hovered and the left mouse button
 * is in the down position.
 *
 * It's recommended that hover casts a drop shadow about 0.5 opacity
 * and down casts the same drop shadows at 0.8 opacity.
 */
public class RDX3DPanelToolbarButton
{
   private Runnable onPressed;
   private RDXIconTexture iconTexture;
   private String tooltipText = null;
   private boolean isHovered = false;
   private boolean isDown = false;
   private final Timer tooltipTimer = new Timer();

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
      iconTexture = new RDXIconTexture(iconAbsoluteResourcePath);
      return iconTexture;
   }

   public void setIconTexture(RDXIconTexture iconTexture)
   {
      this.iconTexture = iconTexture;
   }

   public void onPressed()
   {
      if (onPressed != null)
         onPressed.run();
   }

   public RDXIconTexture getIconTexture()
   {
      return iconTexture;
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

   public Timer getTooltipTimer()
   {
      return tooltipTimer;
   }
}