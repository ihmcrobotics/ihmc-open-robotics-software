package us.ihmc.gdx.ui.tools;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.tools.io.WorkspaceFile;

import java.nio.file.Path;

public class GDXIconTexture
{
   private final Texture texture;

   public GDXIconTexture(WorkspaceFile pngFile)
   {
      this(pngFile.getFilePath());
   }

   public GDXIconTexture(Path pngFile)
   {
      Mat readImage = opencv_imgcodecs.imread(pngFile.toString());
      Pixmap pixmap = new Pixmap(readImage.cols(), readImage.rows(), Pixmap.Format.RGBA8888);
      BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
      Mat rgba8Mat = new Mat(readImage.rows(), readImage.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
      opencv_imgproc.cvtColor(readImage, rgba8Mat, opencv_imgproc.COLOR_RGB2BGRA);
      texture = new Texture(new PixmapTextureData(pixmap, null, false, false));
   }

   public Texture getTexture()
   {
      return texture;
   }
}
