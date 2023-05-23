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
   private static final float ONE_THIRD = 1.0f / 3.0f;
   private static final float TWO_THIRDS = 2.0f / 3.0f;

   private Runnable onPressed;
   private RDXIconTexture iconTexture;
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

   public RDXIconTexture loadAndSetIcons(String iconAbsoluteResourcePath)
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

   public float getUVX0()
   {
      if (isHovered)
         return ONE_THIRD;
      if (isDown)
         return TWO_THIRDS;
      return 0.0f;
   }

   public float getUVX1()
   {
      if (isHovered)
         return TWO_THIRDS;
      if (isDown)
         return 1.0f;
      return ONE_THIRD;
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
}