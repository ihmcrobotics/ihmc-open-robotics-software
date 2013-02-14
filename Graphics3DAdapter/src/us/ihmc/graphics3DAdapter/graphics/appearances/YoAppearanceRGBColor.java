package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Color;

import javax.vecmath.Color3f;

public class YoAppearanceRGBColor extends YoAppearanceTransparency
{
   private final Color3f color; 
   
   public YoAppearanceRGBColor(Color3f color, double transparency)
   {
      this.color = new Color3f(color);
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(Color color, double transparency)
   {
      this.color = new Color3f(color);
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(float red, float green, float blue, double transparency)
   {
      this.color = new Color3f(red, green, blue);
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(double red, double green, double blue, double transparency)
   {
      this.color = new Color3f((float) red, (float) green, (float) blue);
      setTransparency(transparency);
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

   public String toString()
   {
      return "YoAppearanceRGBColor{" + "color=" + color + '}';
   }
}
