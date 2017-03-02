package us.ihmc.graphicsDescription.appearance;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.robotics.dataStructures.MutableColor;

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

   public MutableColor getColor()
   {
      throw new NotImplementedException("getColor() is not implemented");
   }
}
