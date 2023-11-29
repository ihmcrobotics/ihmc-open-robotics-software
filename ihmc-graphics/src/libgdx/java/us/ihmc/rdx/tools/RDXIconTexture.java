package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;

/**
 * Uses OpenCV to load an encoded image (i.e. PNG/JPEG) icon from file or as a resource
 * and writes it to an OpenGL texture to be rendered by libGDX.
 */
public class RDXIconTexture
{
   private final Texture texture;

   /**
    * Load from a file path on disk.
    */
   public RDXIconTexture(Path imagePath)
   {
      Mat readImage = opencv_imgcodecs.imread(imagePath.toString());
      texture = initialize(readImage);
   }

   /**
    * Load from classpath resource path.
    */
   public RDXIconTexture(String iconResourceAbsolutePath)
   {
      if (!iconResourceAbsolutePath.startsWith("/"))
      {
         iconResourceAbsolutePath = "/" + iconResourceAbsolutePath;
      }

      byte[] bytes = ResourceTools.readResourceToByteArray(iconResourceAbsolutePath);

      ByteBuffer directByteBuffer = ByteBuffer.allocateDirect(bytes.length);
      directByteBuffer.order(ByteOrder.nativeOrder());
      directByteBuffer.put(bytes);
      directByteBuffer.rewind();

      BytePointer bytePointer = new BytePointer(directByteBuffer);
      Mat encodedDataMat = new Mat(bytes.length, 1, opencv_core.CV_8UC1, bytePointer);

      // IMREAD_UNCHANGED will keep the 4th channel, the alpha channel; IMREAD_COLOR drops it.
      Mat readImage = opencv_imgcodecs.imdecode(encodedDataMat, opencv_imgcodecs.IMREAD_UNCHANGED);
      texture = initialize(readImage);
   }

   private Texture initialize(Mat readImage)
   {
      Pixmap pixmap = new Pixmap(readImage.cols(), readImage.rows(), Pixmap.Format.RGBA8888);
      BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
      Mat rgba8Mat = new Mat(readImage.rows(), readImage.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
      opencv_imgproc.cvtColor(readImage, rgba8Mat, opencv_imgproc.COLOR_RGB2BGRA);
      Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      return texture;
   }

   public Texture getTexture()
   {
      return texture;
   }
}
