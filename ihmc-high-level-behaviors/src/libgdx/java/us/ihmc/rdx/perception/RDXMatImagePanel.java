package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Gdx2DPixmap;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.rdx.ui.RDXImagePanel;

public class RDXMatImagePanel
{
   private Mat image;
   private Pixmap pixmap;
   private final RDXImagePanel imagePanel;
   private Texture panelTexture;

   public RDXMatImagePanel(String name, int imageWidth, int imageHeight, boolean flipY)
   {
      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      image = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new MutableBytePointer(pixmap.getPixels()));

      imagePanel = new RDXImagePanel(name, flipY);

      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      imagePanel.setTexture(panelTexture);

      OpenCVTools.setRGBA8888ImageAlpha(image, 255);
   }

   public void displayByte(Mat rgbImage)
   {
      if (imagePanel.getIsShowing().get())
      {
         image.copyTo(rgbImage);
         display();
      }
   }

   /**
    * Texture must be drawn to before panel will display the image.
    * This is where the image gets transferred to the GPU.
    */
   public void display()
   {
      if (imagePanel.getIsShowing().get())
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
      imagePanel.setTexture(panelTexture);

      OpenCVTools.setRGBA8888ImageAlpha(image, 255);
   }

   public void ensureDimensionsMatch(int imageWidth, int imageHeight)
   {
      if (!OpenCVTools.dimensionsMatch(image, imageWidth, imageHeight))
      {
         resize(imageWidth, imageHeight);
      }
   }

   public void resize(int imageWidth, int imageHeight)
   {
      panelTexture.dispose();
      pixmap.dispose();

      pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      imagePanel.setTexture(panelTexture);

      image = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new MutableBytePointer(pixmap.getPixels()));

      OpenCVTools.setRGBA8888ImageAlpha(image, 255);
   }

   public RDXImagePanel getImagePanel()
   {
      return imagePanel;
   }

   public Mat getImage()
   {
      return image;
   }
}
