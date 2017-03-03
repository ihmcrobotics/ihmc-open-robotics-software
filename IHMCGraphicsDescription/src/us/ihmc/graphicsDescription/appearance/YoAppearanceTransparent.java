package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.robotics.dataStructures.MutableColor;

public class YoAppearanceTransparent extends YoAppearanceTransparency
{
   public YoAppearanceTransparent()
   {
      setTransparency(1.0);
   }

   @Override
   public MutableColor getColor()
   {
      throw new NotImplementedException("getColor() is not implemented");
   }

   @Override
   public Color getAwtColor()
   {
      throw new NotImplementedException("getAwtColor() is not implemented");
   }
}
