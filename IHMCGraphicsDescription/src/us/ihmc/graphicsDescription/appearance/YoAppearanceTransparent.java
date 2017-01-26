package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import javax.vecmath.Color3f;

import org.apache.commons.lang3.NotImplementedException;

public class YoAppearanceTransparent extends YoAppearanceTransparency
{
   public YoAppearanceTransparent()
   {
      setTransparency(1.0);
   }

   @Override
   public Color3f getColor()
   {
      throw new NotImplementedException("getColor() is not implemented");
   }

   @Override
   public Color getAwtColor()
   {
      throw new NotImplementedException("getAwtColor() is not implemented");
   }
}
