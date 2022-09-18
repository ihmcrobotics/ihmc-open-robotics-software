package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;

public class GDXVideoVisualizer extends ImGuiGDXVisualizer
{
   private final ImPlotFrequencyPlot frequencyPlot = new ImPlotFrequencyPlot("Hz", 30);
   private final ImGuiVideoPanel videoPanel;

   private boolean hasReceivedOne = false;
   private boolean needNewTexture = false;

   private Mat rgba8Mat;
   private BytePointer rgba8888BytePointer;
   private Pixmap pixmap;
   private Texture texture;

   public GDXVideoVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);
      videoPanel = new ImGuiVideoPanel(ImGuiTools.uniqueLabel(this, panelName), flipY);
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

   public void setImage(Mat mat)
   {
      hasReceivedOne = true;
      frequencyPlot.ping();
      updateImageDimensions(mat.cols(), mat.rows());
      rgba8Mat = mat;

      Pixmap pmap = new Pixmap(mat.cols(), mat.rows(), Pixmap.Format.RGBA8888);

      pmap.setColor(Color.GREEN);
      pmap.fill();

      pmap.setColor(Color.RED);
      pmap.fillCircle(80, 80, 40);


      texture = new Texture(pmap);
      pmap.dispose();

      videoPanel.setTexture(texture);
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
}
