package us.ihmc.perception;

import us.ihmc.log.LogTools;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.io.IOException;

public class ImageTools
{
   public static ImageMat loadAsImageMat(String filePath)
   {
      BufferedImage img = null;
      try
      {
         img = ImageIO.read(new File(filePath));
      }
      catch (IOException e)
      {
         LogTools.warn("Could not load image! {}", filePath);
      }
      assert img != null;
      return new ImageMat(((DataBufferByte) (img.getRaster().getDataBuffer())).getData(), img.getHeight(), img.getWidth());
   }

}
