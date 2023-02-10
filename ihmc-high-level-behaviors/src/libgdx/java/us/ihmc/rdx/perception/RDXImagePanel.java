package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Gdx2DPixmap;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

public class RDXImagePanel
{
   private Mat image;
   private Mat normalizedScaledImage;
   private Pixmap pixmap;
   private ImGuiVideoPanel videoPanel;
   private Texture panelTexture;

   public RDXImagePanel(String name, Mat image)
   {
      int imageWidth = image.cols();
      int imageHeight = image.rows();

      this.image = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, image.data());
      normalizedScaledImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);
      createPixmap(imageWidth, imageHeight);

      boolean flipY = false;
      setup(name, imageWidth, imageHeight, flipY);
   }

   public RDXImagePanel(String name, int imageWidth, int imageHeight)
   {
      this(name, imageWidth, imageHeight, false);
   }

   public RDXImagePanel(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      image = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new MutableBytePointer(pixmap.getPixels()));

      setup(name, imageWidth, imageHeight, flipY);
   }

   private void setup(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      videoPanel = new ImGuiVideoPanel(name, flipY);

      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      videoPanel.setTexture(panelTexture);

      BytedecoOpenCVTools.setRGBA8888ImageAlpha(image, 255);
   }

   /**
    * @param singleChannelImage Can be float, unsigned integer, etc
    */
   public void displayFloat(Mat singleChannelImage)
   {
      if (videoPanel.getIsShowing().get())
      {
         BytedecoOpenCVTools.clampTo8BitUnsignedChar(singleChannelImage, normalizedScaledImage, 0.0, 255.0);
         BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage, image);
         display();
      }
   }

   public void displayByte(Mat rgbImage)
   {
      if (videoPanel.getIsShowing().get())
      {
         image.put(rgbImage);
         display();
      }
   }

   /**
    * Texture must be drawn to before panel will display the image.
    * This is where the image gets transferred to the GPU.
    */
   public void display()
   {
      if (videoPanel.getIsShowing().get())
         panelTexture.draw(pixmap, 0, 0);
   }

   private void createPixmap(int imageWidth, int imageHeight)
   {
      long[] nativeData = new long[4];
      nativeData[0] = image.getPointer().address();
      nativeData[1] = imageWidth;
      nativeData[2] = imageHeight;
      nativeData[3] = Gdx2DPixmap.GDX2D_FORMAT_RGBA8888;
      pixmap = new Pixmap(new Gdx2DPixmap(image.getPointer().asByteBuffer(), nativeData));
   }

   public void resize(Mat image)
   {
      int imageWidth = image.cols();
      int imageHeight = image.rows();
      this.image = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, image.data());
      createPixmap(imageWidth, imageHeight);
      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      videoPanel.setTexture(panelTexture);

      normalizedScaledImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);

      BytedecoOpenCVTools.setRGBA8888ImageAlpha(image, 255);
   }

   public void resize(int imageWidth, int imageHeight)
   {
      panelTexture.dispose();
      pixmap.dispose();

      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      videoPanel.setTexture(panelTexture);

      image = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new MutableBytePointer(pixmap.getPixels()));
      normalizedScaledImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1, null);

      BytedecoOpenCVTools.setRGBA8888ImageAlpha(image, 255);
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
   }

   public Pixmap getPixmap()
   {
      return pixmap;
   }
}
