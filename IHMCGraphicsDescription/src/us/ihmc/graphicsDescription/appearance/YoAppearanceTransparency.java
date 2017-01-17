package us.ihmc.graphicsDescription.appearance;

import javax.vecmath.Color3f;

import org.apache.commons.lang3.NotImplementedException;

public abstract class YoAppearanceTransparency implements AppearanceDefinition
{
   private double transparency = 0.0;

   public final double getTransparency()
   {
      return transparency;
   }
   
   public void setTransparency(double transparency)
   {
      this.transparency = transparency;
   }

   public Color3f getColor()
   {
      throw new NotImplementedException("getColor() is not implemented");
   }
}
