package us.ihmc.graphics3DAdapter.graphics.appearances;

import javax.vecmath.Color3f;

public class YoAppearanceRGBColor extends YoAppearanceTransparancy
{
   private final Color3f color; 
   
   public YoAppearanceRGBColor(double red, double green, double blue)
   {
      this((float) red, (float) green, (float) blue);
   }
   
   public YoAppearanceRGBColor(Color3f color)
   {
      this.color = new Color3f(color);
   }
   
   public YoAppearanceRGBColor(float red, float green, float blue)
   {
      this.color = new Color3f(red, green, blue);
   }

   public float getRed()
   {
      return color.getX();
   }

   public float getGreen()
   {
      return color.getY();
   }

   public float getBlue()
   {
      return color.getZ();
   }
   
   public Color3f getColor()
   {
      return color;
   }
}
