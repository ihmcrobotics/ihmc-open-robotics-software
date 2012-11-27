package com.yobotics.simulationconstructionset.graphics;

import javax.vecmath.Color3f;

public class YoAppearanceRGBColor implements YoAppearanceDefinition
{
   private final float red, green, blue;
   
   public YoAppearanceRGBColor(double red, double green, double blue)
   {
      this((float) red, (float) green, (float) blue);
   }
   
   public YoAppearanceRGBColor(Color3f color)
   {
      this(color.x, color.y, color.z);
   }
   
   public YoAppearanceRGBColor(float red, float green, float blue)
   {
      this.red = red;
      this.green = green;
      this.blue = blue;
   }

   public float getRed()
   {
      return red;
   }

   public float getGreen()
   {
      return green;
   }

   public float getBlue()
   {
      return blue;
   }
}
