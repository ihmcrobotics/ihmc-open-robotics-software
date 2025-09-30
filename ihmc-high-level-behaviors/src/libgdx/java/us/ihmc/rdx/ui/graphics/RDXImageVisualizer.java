package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.TextureData;
import com.badlogic.gdx.utils.GdxRuntimeException;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL33;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.tools.thread.SwapReference;

import java.util.concurrent.atomic.AtomicBoolean;

public class RDXImageVisualizer extends RDXVisualizer
{
   private Texture texture;
   private final SwapReference<ImageTextureData> imageDataSwapReference;
   private final AtomicBoolean newImageSet;

   private final RDXImagePanel imagePanel;

   public RDXImageVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);

      imageDataSwapReference = new SwapReference<>(ImageTextureData::new);

      newImageSet = new AtomicBoolean(false);

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

      // Only update the texture if the visualizer is active AND a new image was set
      if (isActive() && newImageSet.getAndSet(false))
      {
         // Update the texture (synchronized over the swap reference to avoid image tearing and other race conditions)
         synchronized (imageDataSwapReference)
         {
            ImageTextureData imageData = imageDataSwapReference.getForThreadOne();

            // If the texture hasn't been created yet, create one
            if (texture == null)
            {
               texture = new Texture(imageData);
               imagePanel.setTexture(texture);
            }
            else
            {
               texture.load(imageData);
            }
         }
      }
   }

   @Override
   public RDXImagePanel getPanel()
   {
      return imagePanel;
   }

   public void setImage(BytePointer imageData, int imageWidth, int imageHeight, long bytesPerPixel, int glColorFormat, int glType)
   {
      ImageTextureData dataToPack = imageDataSwapReference.getForThreadTwo();
      dataToPack.setImage(imageData, imageWidth, imageHeight, bytesPerPixel, glColorFormat, glType);
      newImageSet.set(true);
      imageDataSwapReference.swap();
   }

   public void setImage(Mat mat, PixelFormat pixelFormat)
   {
      // Ensure the pixel format is compatible with OpenGL color formats
      Mat compatibleColorMat;
      int colorFormat = pixelFormat.toOpenGLColorFormat();
      if (colorFormat >= 0) // If compatible use the passed in Mat directly
      {
         compatibleColorMat = mat;
      }
      else // If incompatible attempt to convert to RGBA
      {
         compatibleColorMat = new Mat();
         if (!pixelFormat.convertToRGBA(mat, compatibleColorMat))
            throw new RuntimeException(pixelFormat.name() + " is incompatible with OpenGL and conversion to RGBA failed.");

         colorFormat = GL33.GL_RGBA;
      }

      // Ensure continuity of data
      Mat continuousMat;
      if (compatibleColorMat.isContinuous())
         continuousMat = compatibleColorMat;
      else
         continuousMat = compatibleColorMat.clone();

      // Get the OpenGL type based on OpenCV depth type
      int depthType = continuousMat.type() & opencv_core.CV_MAT_DEPTH_MASK;
      int glFormat = switch (depthType)
      {
         case opencv_core.CV_8U ->  GL33.GL_UNSIGNED_BYTE;
         case opencv_core.CV_8S ->  GL33.GL_BYTE;
         case opencv_core.CV_16U -> GL33.GL_UNSIGNED_SHORT;
         case opencv_core.CV_16S -> GL33.GL_SHORT;
         case opencv_core.CV_32S -> GL33.GL_INT;
         case opencv_core.CV_32F -> GL33.GL_FLOAT;
         case opencv_core.CV_64F -> GL33.GL_DOUBLE;
         case opencv_core.CV_16F -> GL33.GL_HALF_FLOAT;
         default -> throw new IllegalStateException("No such depth type: " + depthType);
      };

      // Set the image
      setImage(continuousMat.data(), continuousMat.cols(), continuousMat.rows(), continuousMat.elemSize(), colorFormat, glFormat);

      // Deallocate the Mats if new ones were allocated
      if (continuousMat != compatibleColorMat)
         continuousMat.close();
      if (compatibleColorMat != mat)
         compatibleColorMat.close();
   }

   public void setImage(RawImage image)
   {
      if (image.get() == null)
         return;

      setImage(image.getCpuImageMat(), image.getPixelFormat());

      image.release();
   }

   public Texture getTexture()
   {
      return texture;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (texture != null)
         texture.dispose();

      imageDataSwapReference.getA().dispose();
      imageDataSwapReference.getB().dispose();
   }

   private static class ImageTextureData implements TextureData
   {
      private BytePointer dataPointer = new BytePointer();
      private int width = 0;
      private int height = 0;
      private int colorFormat = GL33.GL_RGBA;
      private int type = GL33.GL_UNSIGNED_BYTE;

      private boolean isPrepared = false;

      public void setImage(BytePointer imageData, int imageWidth, int imageHeight, long bytesPerPixel, int glColorFormat, int glType)
      {
         // Set the metadata
         width = imageWidth;
         height = imageHeight;
         colorFormat = glColorFormat;
         type = glType;

         // Ensure data pointer allocation is correct size
         long imageSize = bytesPerPixel * imageWidth * imageHeight;
         if (dataPointer.capacity() != imageSize)
         {
            dataPointer.close();
            dataPointer = new BytePointer(imageSize);
         }

         // Copy image data to the pointer location
         BytePointer.memcpy(dataPointer, imageData, imageSize);
      }

      public void dispose()
      {
         dataPointer.close();
      }

      @Override
      public TextureDataType getType()
      {
         return TextureDataType.Custom;
      }

      @Override
      public boolean isPrepared()
      {
         return isPrepared;
      }

      @Override
      public void prepare()
      {
         if (isPrepared)
            throw new GdxRuntimeException("Already Prepared");
         else
            isPrepared = true;
      }

      @Override
      public Pixmap consumePixmap()
      {
         throw new GdxRuntimeException("This TextureData implementation does not return a Pixmap");
      }

      @Override
      public boolean disposePixmap()
      {
         throw new GdxRuntimeException("This TextureData implementation does not return a Pixmap");
      }

      @Override
      public void consumeCustomData(int target)
      {
         // If the color format is monochrome (single color), render it in gray scale
         if (colorFormat == GL33.GL_RED)
         {
            GL33.glTexParameteri(GL33.GL_TEXTURE_2D, GL33.GL_TEXTURE_SWIZZLE_G, GL33.GL_RED);
            GL33.glTexParameteri(GL33.GL_TEXTURE_2D, GL33.GL_TEXTURE_SWIZZLE_B, GL33.GL_RED);
         }

         // Get a good internal color format
         int internalFormat = switch (colorFormat)
         {
            case GL33.GL_BGR -> GL33.GL_RGB;
            case GL33.GL_BGRA -> GL33.GL_RGBA;
            default -> colorFormat;
         };

         GL33.glTexImage2D(target, 0, internalFormat, width, height, 0, colorFormat, type, dataPointer.address());
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
      public Format getFormat()
      {
         return switch (colorFormat)
         {
            case GL33.GL_RGB, GL33.GL_BGR -> Format.RGB888;
            case GL33.GL_RGBA, GL33.GL_BGRA -> Format.RGBA8888;
            case GL33.GL_RED -> Format.Alpha;
            default -> throw new GdxRuntimeException("Color format not implemented");
         };
      }

      @Override
      public boolean useMipMaps()
      {
         return false;
      }

      @Override
      public boolean isManaged()
      {
         return false;
      }
   }
}
