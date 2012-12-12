package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Color;

import javax.vecmath.Color3f;

public class YoAppearanceRGBColor extends YoAppearanceTransparancy
{
   private final Color3f color; 
   
   public YoAppearanceRGBColor(Color3f color, double transparancy)
   {
      this.color = new Color3f(color);
      this.transparancy = transparancy;
   }
   
   public YoAppearanceRGBColor(Color color, double transparancy)
   {
      this.color = new Color3f(color);
      this.transparancy = transparancy;
   }
   
   public YoAppearanceRGBColor(float red, float green, float blue, double transparancy)
   {
      this.color = new Color3f(red, green, blue);
      this.transparancy = transparancy;
   }
   
   public YoAppearanceRGBColor(double red, double green, double blue, double transparancy)
   {
      this.color = new Color3f((float) red, (float) green, (float) blue);
      this.transparancy = transparancy;
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
