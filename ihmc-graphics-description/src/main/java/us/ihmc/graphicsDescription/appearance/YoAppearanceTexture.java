package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;
import java.awt.image.BufferedImage;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.robotics.dataStructures.MutableColor;

public class YoAppearanceTexture extends YoAppearanceTransparency
{
   private final String path;
   private final BufferedImage bufferedImage;

   public YoAppearanceTexture(String path)
   {
      super();
      this.path = path;
      this.bufferedImage = null;
   }

   public YoAppearanceTexture(BufferedImage bufferedImage)
   {
      super();
      this.path = null;
      this.bufferedImage = bufferedImage;
   }

   public String getPath()
   {
      return path;
   }
   
   public BufferedImage getBufferedImage()
   {
      return bufferedImage;
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
