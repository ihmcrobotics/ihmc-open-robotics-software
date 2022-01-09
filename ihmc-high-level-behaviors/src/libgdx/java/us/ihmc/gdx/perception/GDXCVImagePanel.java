package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.perception.BytedecoOpenCVTools;

public class GDXCVImagePanel
{
   private final ImGuiVideoPanel videoPanel;
   private final GDXBytedecoImage bytedecoImage;
   private final Pixmap pixmap;
   private final Texture panelTexture;

   private final GDXBytedecoImage normalizedScaledImage;

   public GDXCVImagePanel(String name, int imageWidth, int imageHeight)
   {
      videoPanel = new ImGuiVideoPanel(name, true);
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      bytedecoImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());
      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      videoPanel.setTexture(panelTexture);

      normalizedScaledImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);

      BytedecoOpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
   }

   public void drawFloatImage(Mat floatImage)
   {
      BytedecoOpenCVTools.clamp32BitFloatTo8BitUnsignedChar(floatImage, normalizedScaledImage.getBytedecoOpenCVMat(), 0.0, 255.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage.getBytedecoOpenCVMat(), bytedecoImage.getBytedecoOpenCVMat());
      draw();
   }

   /**
    * Texture must be drawn to before panel will display the image.
    * This is where the image gets transferred to the GPU.
    */
   public void draw()
   {
      panelTexture.draw(pixmap, 0, 0);
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
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
