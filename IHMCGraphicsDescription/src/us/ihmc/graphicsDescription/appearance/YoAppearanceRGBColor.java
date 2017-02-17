package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import us.ihmc.robotics.dataStructures.MutableColor;

public class YoAppearanceRGBColor extends YoAppearanceTransparency
{
   private final MutableColor color; 
   private final Color awtColor;
   
   public YoAppearanceRGBColor(MutableColor color, double transparency)
   {
      this.color = new MutableColor(color);
      awtColor = new Color(this.color.getX(), this.color.getY(), this.color.getZ(), (float) (1.0 - transparency));
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(Color color, double transparency)
   {
      this.color = new MutableColor(color);
      awtColor = new Color(color.getRed() / 255.0f, color.getGreen() / 255.0f, color.getBlue() / 255.0f, (float) (1.0 - transparency));
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(float red, float green, float blue, double transparency)
   {
      color = new MutableColor(red, green, blue);
      awtColor = new Color(red, green, blue, (float) (1.0 - transparency));
      setTransparency(transparency);
   }
   
   public YoAppearanceRGBColor(double red, double green, double blue, double transparency)
   {
      color = new MutableColor((float) red, (float) green, (float) blue);
      awtColor = new Color((float) red, (float) green, (float) blue, (float) (1.0 - transparency));
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
   public MutableColor getColor()
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
