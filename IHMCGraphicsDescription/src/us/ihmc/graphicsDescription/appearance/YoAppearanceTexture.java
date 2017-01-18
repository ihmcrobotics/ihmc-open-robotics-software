package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;
import java.awt.image.BufferedImage;

import javax.vecmath.Color3f;

import org.apache.commons.lang3.NotImplementedException;

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
