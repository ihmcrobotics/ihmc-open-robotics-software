package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Color;

import javax.vecmath.Color3f;

public class YoAppearanceRGBColor extends YoAppearanceTransparency
{
   private final Color3f color; 
   private final Color awtColor;
   
   public YoAppearanceRGBColor(Color3f color, double transparency)
   {
      this.color = new Color3f(color);
      this.awtColor = new Color(this.color.x, this.color.y, this.color.z, (float) transparency);
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(Color color, double transparency)
   {
      this.color = new Color3f(color);
      this.awtColor = new Color(color.getRed(), color.getGreen(), color.getBlue(), (float) transparency);
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(float red, float green, float blue, double transparency)
   {
      this.color = new Color3f(red, green, blue);
      this.awtColor = new Color(red / 255.0f, green / 255.0f, blue / 255.0f, (float) transparency);
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(double red, double green, double blue, double transparency)
   {
      this.color = new Color3f((float) red, (float) green, (float) blue);
      this.awtColor = new Color((float) (red / 255.0), (int) (green / 255.0), (float) (blue / 255.0), (float) transparency);
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
   
   @Override
   public Color3f getColor()
   {
      return color;
   }

   @Override
   public Color getAwtColor()
   {
      return awtColor;
   }

   @Override
   public String toString()
   {
      return "YoAppearanceRGBColor{" + "color=" + color + '}';
   }
}
