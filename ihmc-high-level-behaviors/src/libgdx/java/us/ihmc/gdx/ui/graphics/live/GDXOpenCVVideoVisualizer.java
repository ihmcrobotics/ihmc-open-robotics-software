package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class GDXOpenCVVideoVisualizer extends ImGuiGDXVisualizer
{
   private Mat rgba8Mat;
   private boolean needNewTexture = false;
   private BytePointer rgba8888BytePointer;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();

   private final ResettableExceptionHandlingExecutorService threadQueue;
   private Pixmap pixmap;
   private Texture texture;
   private final ImGuiVideoPanel videoPanel;

   public GDXOpenCVVideoVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      videoPanel = new ImGuiVideoPanel(ImGuiTools.uniqueLabel(this, panelName), flipY);
   }

   protected void doReceiveMessageOnThread(Runnable receiveMessageOnThread)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         getThreadQueue().clearQueueAndExecute(receiveMessageOnThread);
      }
   }

   protected void updateImageDimensions(int imageWidth, int imageHeight)
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
                  videoPanel.setTexture(texture);
               }

               texture.draw(pixmap, 0, 0);
            }
         }
      }
   }

   @Override
   public ImGuiVideoPanel getPanel()
   {
      return videoPanel;
   }

   protected Mat getRGBA8Mat()
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

   protected ImGuiFrequencyPlot getFrequencyPlot()
   {
      return frequencyPlot;
   }
}
