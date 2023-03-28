package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.perception.BytedecoImage;

/**
 * This class will be instantiated twice for each panel. There will be
 * two sets of pixmaps, textures, and images where each set shares some
 * native memory, that gets copied to the GPU in the UI update method.
 */
public class RDXImagePanelTexture
{
   private Pixmap pixmap;
   private boolean needNewTexture = false;
   private Texture texture;
   private BytedecoImage bytedecoImage;

   /**
    * Ensure the texture is allocated and is the correct dimensions.
    * Does not require active OpenGL context.
    */
   public void ensureTextureDimensions(int imageWidth, int imageHeight)
   {
      if (pixmap == null || pixmap.getWidth() != imageWidth || pixmap.getHeight() != imageHeight)
      {
         if (pixmap != null)
         {
            pixmap.dispose();
         }

         pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
         bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());
         needNewTexture = true;
      }
   }

   /**
    * Meant to be called from the RDX render thread with an active
    * OpenGL context. This does what we can't do asynchronously:
    * creating textures and copying textures to the GPU.
    */
   public void updateTextureAndDraw(RDXImagePanel imagePanel)
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
         }

         texture.draw(pixmap, 0, 0); // Copies from RAM to the GPU

         imagePanel.setTexture(texture);
      }
   }

   /** For convenient OpenCV usage. */
   public Mat getRGBA8Mat()
   {
      return bytedecoImage.getBytedecoOpenCVMat();
   }

   public BytedecoImage getRGBA8Image()
   {
      return bytedecoImage;
   }
}
