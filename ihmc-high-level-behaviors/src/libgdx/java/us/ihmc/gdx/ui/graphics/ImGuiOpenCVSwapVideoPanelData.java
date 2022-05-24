package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;

public class ImGuiOpenCVSwapVideoPanelData
{
   private Pixmap pixmap;
   private BytePointer rgba8888BytePointer;
   private Mat rgba8Mat;
   private boolean needNewTexture = false;
   private Texture texture;

   public void updateOnImageUpdateThread(int imageWidth, int imageHeight)
   {
      if (pixmap == null || pixmap.getWidth() != imageWidth || pixmap.getHeight() != imageHeight)
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

   public void updateOnUIThread(ImGuiVideoPanel videoPanel)
   {
      if (pixmap != null)
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

   public Mat getRGBA8Mat()
   {
      return rgba8Mat;
   }
}
