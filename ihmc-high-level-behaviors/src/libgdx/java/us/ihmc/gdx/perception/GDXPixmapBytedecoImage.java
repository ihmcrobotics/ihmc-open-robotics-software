package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import org.bytedeco.opencv.global.opencv_core;

public class GDXPixmapBytedecoImage
{
   private final GDXBytedecoImage bytedecoImage;
   private final Pixmap pixmap;

   public GDXPixmapBytedecoImage(int imageWidth, int imageHeight)
   {
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      pixmap.getPixels().rewind();
      bytedecoImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());
   }

   public GDXBytedecoImage getBytedecoImage()
   {
      return bytedecoImage;
   }

   public Pixmap getPixmap()
   {
      return pixmap;
   }
}
