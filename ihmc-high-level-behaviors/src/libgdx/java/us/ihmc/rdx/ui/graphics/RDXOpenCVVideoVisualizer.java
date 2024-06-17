package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class RDXOpenCVVideoVisualizer extends RDXVisualizer
{
   private Mat rgba8Mat;
   private boolean needNewTexture = false;
   private BytePointer rgba8888BytePointer;
   private boolean hasRenderedOne = false;

   private final ResettableExceptionHandlingExecutorService threadQueue;
   private Pixmap pixmap;
   private Texture texture;
   private final RDXImagePanel imagePanel;
   private final List<Consumer<Mat>> overlays = new ArrayList<>();

   public RDXOpenCVVideoVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      imagePanel = new RDXImagePanel(ImGuiTools.uniqueLabel(this, panelName), flipY);
   }

   public void doReceiveMessageOnThread(Runnable receiveMessageOnThread)
   {
      hasRenderedOne = true;
      if (isActive())
      {
         getThreadQueue().clearQueueAndExecute(receiveMessageOnThread);
      }
   }

   public void updateImageDimensions(int imageWidth, int imageHeight)
   {
      if (rgba8Mat == null || pixmap.getWidth() != imageWidth || pixmap.getHeight() != imageHeight)
      {
         if (pixmap != null)
         {
            pixmap.dispose();
         }

         pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
         rgba8888BytePointer = new BytePointer(pixmap.getPixels());
         rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, rgba8888BytePointer);
         needNewTexture = true;
      }
   }

   public void setImage(Mat mat)
   {
      hasRenderedOne = true;
      updateImageDimensions(mat.cols(), mat.rows());
      opencv_imgproc.cvtColor(mat, rgba8Mat, opencv_imgproc.COLOR_RGB2RGBA);
   }

   @Override
   public void renderImGuiWidgets()
   {

   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         synchronized (this)
         {
            if (rgba8Mat != null)
            {
               if (texture == null || needNewTexture)
               {
                  needNewTexture = false;
                  if (texture != null)
                  {
                     texture.dispose();
                  }

                  texture = new Texture(new PixmapTextureData(pixmap, null, false, false));
                  imagePanel.setTexture(texture);
               }

               for (Consumer<Mat> overlay : overlays)
               {
                  overlay.accept(rgba8Mat);
               }

               texture.draw(pixmap, 0, 0);
            }
         }
      }
   }

   @Override
   public RDXImagePanel getPanel()
   {
      return imagePanel;
   }

   /** An overlay allows users to draw over this image using OpenCV calls before it gets rendered. */
   public void addOverlay(Consumer<Mat> overlay)
   {
      overlays.add(overlay);
   }

   public Mat getRGBA8Mat()
   {
      return rgba8Mat;
   }

   public BytePointer getRgba8888BytePointer()
   {
      return rgba8888BytePointer;
   }

   protected ResettableExceptionHandlingExecutorService getThreadQueue()
   {
      return threadQueue;
   }

   public boolean getHasRenderedOne()
   {
      return hasRenderedOne;
   }

   public Texture getTexture()
   {
      return texture;
   }
}
