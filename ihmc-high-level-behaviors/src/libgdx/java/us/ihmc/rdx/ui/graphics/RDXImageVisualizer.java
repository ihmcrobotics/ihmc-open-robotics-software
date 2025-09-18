package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.GLTexture;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.tools.thread.GuidedSwapReference;
import us.ihmc.tools.thread.SwapReference;

import java.util.concurrent.atomic.AtomicBoolean;

public class RDXImageVisualizer extends RDXVisualizer
{
   // TODO: Finish converting to swap reference
   private final SwapReference<Mat> rgbaSwapReference = new SwapReference<>(Mat::new);
   private Mat rgbaMat;
   private MatTexture texture;
   private final AtomicBoolean newImageReceived;

   private final RDXImagePanel imagePanel;

   public RDXImageVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);

      newImageReceived = new AtomicBoolean(false);

      ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
      imagePanel = new RDXImagePanel(labels.get(panelName), flipY);
   }

   @Override
   public void renderImGuiWidgets()
   {

   }

   @Override
   public void update()
   {
      super.update();
      if (isActive() && newImageReceived.getAndSet(false))
      {
         if (texture == null)
         {
            texture = new MatTexture();
            imagePanel.setTexture(texture);
         }

         texture.prepareForUpload();
         synchronized (this)
         {
            texture.upload(rgbaMat);
         }
      }
   }

   @Override
   public RDXImagePanel getPanel()
   {
      return imagePanel;
   }

   public void updateImageDimensions(int imageWidth, int imageHeight)
   {
      if (rgbaMat == null || imageWidth != rgbaMat.cols() || imageHeight != rgbaMat.rows())
      {
         synchronized (this)
         {
            // Dispose of objects that are about to be reallocated
            if (rgbaMat != null)
               rgbaMat.close();

            rgbaMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4);
         }
      }
   }

   public Mat getRGBAMat()
   {
      return rgbaMat;
   }

   public void setImage(Mat rgbMat)
   {
      setImage(rgbMat, opencv_imgproc.COLOR_RGB2RGBA);
   }

   public void setImage(Mat mat, int toRGBAConversion)
   {
      synchronized (this)
      {
         updateImageDimensions(mat.cols(), mat.rows());
         opencv_imgproc.cvtColor(mat, rgbaMat, toRGBAConversion);
      }
      newImageReceived.set(true);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      synchronized (this)
      {
         if (texture != null)
            texture.dispose();
         if (rgbaMat != null)
            rgbaMat.close();
      }
   }

   private static class MatTexture extends GLTexture
   {
      private int width = 0;
      private int height = 0;

      public MatTexture()
      {
         super(GL41.GL_TEXTURE_2D);
      }

      @Override
      public int getWidth()
      {
         return width;
      }

      @Override
      public int getHeight()
      {
         return height;
      }

      @Override
      public int getDepth()
      {
         return 0;
      }

      @Override
      public boolean isManaged()
      {
         return false;
      }

      @Override
      protected void reload()
      {
         throw new RuntimeException("I don't know how to reload :(");
      }

      public void prepareForUpload()
      {
         bind();
         GL41.glTexParameteri(GL41.GL_TEXTURE_2D, GL41.GL_TEXTURE_MIN_FILTER, GL41.GL_LINEAR);
         GL41.glTexParameteri(GL41.GL_TEXTURE_2D, GL41.GL_TEXTURE_MAG_FILTER, GL41.GL_LINEAR);
      }

      public void upload(Mat mat)
      {
         width = mat.cols();
         height = mat.rows();
         GL41.glTexImage2D(glTarget, 0, GL41.GL_RGBA, width, height, 0, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE, mat.data().address());
      }
   }
}
