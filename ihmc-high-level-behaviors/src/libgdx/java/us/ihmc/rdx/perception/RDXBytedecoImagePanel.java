package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Gdx2DPixmap;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;

/**
 * Possible to render grayscale images directly? Pixmap format Alpha texture?
 */
public class RDXBytedecoImagePanel
{
   private Pixmap pixmap;
   private final BytedecoImage bytedecoImage;
   private RDXImagePanel imagePanel;
   private Texture panelTexture;

   private BytedecoImage normalizedScaledImage;

   public RDXBytedecoImagePanel(String name, BytedecoImage bytedecoImage)
   {
      int imageWidth = bytedecoImage.getImageWidth();
      int imageHeight = bytedecoImage.getImageHeight();

      this.bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, bytedecoImage.getBackingDirectByteBuffer());
      createPixmapFromBytedecoImage(imageWidth, imageHeight);

      setup(name, imageWidth, imageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
   }

   public RDXBytedecoImagePanel(String name, int imageWidth, int imageHeight)
   {
      this(name, imageWidth, imageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
   }

   public RDXBytedecoImagePanel(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());

      setup(name, imageWidth, imageHeight, flipY);
   }

   private void setup(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      imagePanel = new RDXImagePanel(name, flipY);

      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      imagePanel.setTexture(panelTexture);

      normalizedScaledImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);

      OpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
   }

   /**
    * @param singleChannelImage Can be float, unsigned integer, etc
    */
   public void drawDepthImage(Mat singleChannelImage)
   {
      if (imagePanel.getIsShowing().get())
      {
         OpenCVTools.clampTo8BitUnsignedChar(singleChannelImage, normalizedScaledImage.getBytedecoOpenCVMat(), 0.0, 255.0);
         OpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage.getBytedecoOpenCVMat(), bytedecoImage.getBytedecoOpenCVMat());
         draw();
      }
   }

   /**
    * @param colorImage Color image to display
    */
   public void drawColorImage(Mat colorImage)
   {
      if (imagePanel.getIsShowing().get())
      {
         opencv_imgproc.cvtColor(colorImage, bytedecoImage.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_BGRA2RGBA);
         draw();
      }
   }

   /**
    * Draw a Mat image to the panel that can be any size.
    */
   public void drawResizeAndCopy(Mat sourceImage)
   {
      resize(sourceImage.cols(), sourceImage.rows(), null);
      sourceImage.copyTo(bytedecoImage.getBytedecoOpenCVMat());
      draw();
   }

   /**
    * Texture must be drawn to before panel will display the image.
    * This is where the image gets transferred to the GPU.
    */
   public void draw()
   {
      bytedecoImage.rewind();

      if (imagePanel.getIsShowing().get())
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

   public void resize(BytedecoImage bytedecoImage)
   {
      if (!OpenCVTools.dimensionsMatch(this.bytedecoImage, bytedecoImage))
      {
         int imageWidth = bytedecoImage.getImageWidth();
         int imageHeight = bytedecoImage.getImageHeight();
         this.bytedecoImage.resize(imageWidth, imageHeight, null, bytedecoImage.getBackingDirectByteBuffer());
         createPixmapFromBytedecoImage(imageWidth, imageHeight);
         panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
         imagePanel.setTexture(panelTexture);

         normalizedScaledImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);

         OpenCVTools.setRGBA8888ImageAlpha(this.bytedecoImage.getBytedecoOpenCVMat(), 255);
      }
   }

   public void resize(int imageWidth, int imageHeight, OpenCLManager openCLManager)
   {
      panelTexture.dispose();
      pixmap.dispose();

      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      imagePanel.setTexture(panelTexture);

      bytedecoImage.resize(imageWidth, imageHeight, openCLManager, pixmap.getPixels());
      normalizedScaledImage.resize(imageWidth, imageHeight, openCLManager, null);

      OpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
   }

   public void updateDataAddress(long address)
   {
      bytedecoImage.rewind();
      bytedecoImage.changeAddress(address);
      createPixmapFromBytedecoImage(bytedecoImage.getImageWidth(), bytedecoImage.getImageHeight());
   }

   public RDXImagePanel getImagePanel()
   {
      return imagePanel;
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
