package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.image.BufferedImage;
import java.net.URL;

public class YoAppearanceTexture extends YoAppearanceTransparency
{
   private final URL fileURL;
   private final BufferedImage bufferedImage;

   public YoAppearanceTexture(URL fileURL)
   {
      super();
      this.fileURL = fileURL;
      this.bufferedImage = null;
   }

   public YoAppearanceTexture(BufferedImage bufferedImage)
   {
      super();
      this.fileURL = null;
      this.bufferedImage = bufferedImage;
   }

   public URL getFileURL()
   {
      return fileURL;
   }
   
   public BufferedImage getBufferedImage()
   {
      return bufferedImage;
   }

}
