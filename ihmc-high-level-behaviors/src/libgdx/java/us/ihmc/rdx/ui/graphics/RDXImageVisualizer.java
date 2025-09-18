package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.GLTexture;
import org.bytedeco.javacpp.BytePointer;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.tools.thread.SwapReference;

import java.util.concurrent.atomic.AtomicBoolean;

public class RDXImageVisualizer extends RDXVisualizer
{
   private DirectTexture2D texture;
   private final SwapReference<ImageData> imageDataSwapReference;
   private final AtomicBoolean newImageReceived;

   private final RDXImagePanel imagePanel;

   public RDXImageVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);

      imageDataSwapReference = new SwapReference<>(ImageData::new);

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
            texture = new DirectTexture2D();
            imagePanel.setTexture(texture);
         }

         texture.prepareForUpload();
         synchronized (imageDataSwapReference)
         {
            ImageData imageData = imageDataSwapReference.getForThreadOne();
            texture.upload(imageData.dataPointer, imageData.width, imageData.height);
         }
      }
   }

   @Override
   public RDXImagePanel getPanel()
   {
      return imagePanel;
   }

   public void setImage(BytePointer imageData, int imageWidth, int imageHeight)
   {
      ImageData dataToPack = imageDataSwapReference.getForThreadTwo();
      dataToPack.width = imageWidth;
      dataToPack.height = imageHeight;

      int imageSize = 4 * imageWidth * imageHeight; // RGBA - 4 bytes per pixel
      if (dataToPack.dataPointer == null || dataToPack.dataPointer.capacity() != imageSize)
      {
         if (dataToPack.dataPointer != null)
            dataToPack.dataPointer.close();
         dataToPack.dataPointer = new BytePointer(imageSize);
      }

      BytePointer.memcpy(dataToPack.dataPointer, imageData, imageSize);
      imageDataSwapReference.swap();
      newImageReceived.set(true);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (texture != null)
         texture.dispose();
      if (imageDataSwapReference.getA().dataPointer != null)
         imageDataSwapReference.getA().dataPointer.close();
      if (imageDataSwapReference.getB().dataPointer != null)
         imageDataSwapReference.getB().dataPointer.close();
   }

   private static class ImageData
   {
      public BytePointer dataPointer = null;
      public int width = 0;
      public int height = 0;
   }

   private static class DirectTexture2D extends GLTexture
   {
      private int width = 0;
      private int height = 0;

      public DirectTexture2D()
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

      public void upload(BytePointer rgbaData, int width, int height)
      {
         this.width = width;
         this.height = height;

         GL41.glTexImage2D(glTarget, 0, GL41.GL_RGBA, width, height, 0, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE, rgbaData.address());
      }
   }
}
