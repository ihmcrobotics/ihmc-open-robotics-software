package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Gdx2DPixmap;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;

/**
 * Possible to render grayscale images directly? Pixmap format Alpha texture?
 */
public class GDXCVImagePanel
{
   private Pixmap pixmap;
   private final BytedecoImage bytedecoImage;
   private ImGuiVideoPanel videoPanel;
   private Texture panelTexture;

   private BytedecoImage normalizedScaledImage;

   public GDXCVImagePanel(String name, BytedecoImage bytedecoImage)
   {
      int imageWidth = bytedecoImage.getImageWidth();
      int imageHeight = bytedecoImage.getImageHeight();

      this.bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, bytedecoImage.getBackingDirectByteBuffer());
      createPixmapFromBytedecoImage(imageWidth, imageHeight);

      boolean flipY = false;
      setup(name, imageWidth, imageHeight, flipY);
   }

   public GDXCVImagePanel(String name, int imageWidth, int imageHeight)
   {
      this(name, imageWidth, imageHeight, false);
   }

   public GDXCVImagePanel(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());

      setup(name, imageWidth, imageHeight, flipY);
   }

   private void setup(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      videoPanel = new ImGuiVideoPanel(name, flipY);

      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      videoPanel.setTexture(panelTexture);

      normalizedScaledImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);

      BytedecoOpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
   }

   public void drawFloatImage(Mat floatImage)
   {
      if (videoPanel.getIsShowing().get())
      {
         BytedecoOpenCVTools.clampTo8BitUnsignedChar(floatImage, normalizedScaledImage.getBytedecoOpenCVMat(), 0.0, 255.0);
         BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage.getBytedecoOpenCVMat(), bytedecoImage.getBytedecoOpenCVMat());
         draw();
      }
   }

   /**
    * Texture must be drawn to before panel will display the image.
    * This is where the image gets transferred to the GPU.
    */
   public void draw()
   {
      bytedecoImage.rewind();

      if (videoPanel.getIsShowing().get())
         panelTexture.draw(pixmap, 0, 0);
   }

   private void createPixmapFromBytedecoImage(int imageWidth, int imageHeight)
   {
      long[] nativeData = new long[4];
      nativeData[0] = bytedecoImage.getBytedecoByteBufferPointer().address();
      nativeData[1] = imageWidth;
      nativeData[2] = imageHeight;
      nativeData[3] = Gdx2DPixmap.GDX2D_FORMAT_RGBA8888;
      pixmap = new Pixmap(new Gdx2DPixmap(bytedecoImage.getBackingDirectByteBuffer(), nativeData));
   }

   public void resize(int imageWidth, int imageHeight, OpenCLManager openCLManager)
   {
      panelTexture.dispose();
      pixmap.dispose();

      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      videoPanel.setTexture(panelTexture);

      bytedecoImage.resize(imageWidth, imageHeight, openCLManager, pixmap.getPixels());
      normalizedScaledImage.resize(imageWidth, imageHeight, openCLManager, null);

      BytedecoOpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
   }

   public void updateDataAddress(long address)
   {
      bytedecoImage.rewind();
      bytedecoImage.changeAddress(address);
      createPixmapFromBytedecoImage(bytedecoImage.getImageWidth(), bytedecoImage.getImageHeight());
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
   }

   public BytedecoImage getBytedecoImage()
   {
      return bytedecoImage;
   }

   public Pixmap getPixmap()
   {
      return pixmap;
   }
}
