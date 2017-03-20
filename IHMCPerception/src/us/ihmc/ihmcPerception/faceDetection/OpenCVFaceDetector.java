package us.ihmc.ihmcPerception.faceDetection;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

import javax.imageio.ImageIO;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.videoio.VideoCapture;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class OpenCVFaceDetector
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }
   
   private static final boolean DEBUG = false;
   private static final String HAARCASCADE_FRONTALFACE_ALT_XML = "faceDetection/haarcascade_frontalface_alt.xml";

   private CascadeClassifier cascadeClassifierForFaces;
   private final MatOfRect faces = new MatOfRect();
   private final double scaleFactor;

   /**
    * @param scaleFactor scaleDownImage for faster processing (scaleFactor <1.0)
    */
   public OpenCVFaceDetector(double scaleFactor)
   {
      this.scaleFactor = scaleFactor;
      
      try
      {
         URI uri = ClassLoader.getSystemResource(HAARCASCADE_FRONTALFACE_ALT_XML).toURI();
         Map<String, String> env = new HashMap<>();
         env.put("create", "true");
         FileSystem zipfs = FileSystems.newFileSystem(uri, env);
         Path xmlPath = Paths.get(uri);
         cascadeClassifierForFaces = new CascadeClassifier(xmlPath.toString());

         if (cascadeClassifierForFaces.empty())
         {
            throw new RuntimeException("cascadeClassifier is empty");
         }
      }
      catch (URISyntaxException | IOException e)
      {
         cascadeClassifierForFaces = null;
         e.printStackTrace();
      }
   }
   
   int count = 0;
   Stopwatch timer = null;
   double conversion = 0.0;
   double resize = 0.0;
   double detection = 0.0;
   
   {
      if (DEBUG)
      {
         timer = new Stopwatch().start();
      }
   }

   public Rect[] detect(BufferedImage bufferedImage)
   {
      if (DEBUG) timer.lap();
      Mat image = OpenCVTools.convertBufferedImageToMat(bufferedImage);
      if (DEBUG) conversion = timer.lap();
      Imgproc.resize(image, image, new Size((int) (image.width() * scaleFactor), (int) (image.height() * scaleFactor)));
      if (DEBUG) resize = timer.lap();
      cascadeClassifierForFaces.detectMultiScale(image, faces);
      if (DEBUG)
      {
         detection = timer.lap();
         PrintTools.info("Conversion: " + conversion + " Resize: " + resize + " Detection: " + detection);

         if (count++ % 100 == 0)
         {
            PrintTools.info("Writing image: " + Imgcodecs.imwrite("/home/shadylady/image.jpeg", image));

            for (Rect rect : faces.toArray())
            {
               PrintTools.info("Rect: x:" + rect.x + " y:" + rect.y + " w:" + rect.width + " h:" + rect.height);
            }
         }
      }
      
      Rect[] faceArray = faces.toArray();
      for (Rect face : faceArray)
      {
         face.x = (int) (face.x / scaleFactor);
         face.y = (int) (face.y / scaleFactor);
         face.width = (int) (face.width / scaleFactor);
         face.height = (int) (face.height / scaleFactor);
      }
      return faceArray;
   }

   public Rect[] detectAndOutline(BufferedImage bufferedImage)
   {
      Rect[] faces = detect(bufferedImage);
      Graphics2D g2 = bufferedImage.createGraphics();

      for (int i = 0; i < faces.length; i++)
      {
         if (DEBUG)
         {
            System.out.println("Face " + i + ": " + faces[i]);
         }
         g2.drawRect(faces[i].x, faces[i].y, faces[i].width, faces[i].height);
      }

      g2.finalize();

      return faces;
   }

   public static void main(String[] arg) throws IOException
   {      
      VideoCapture cap = new VideoCapture(0);
      OpenCVFaceDetector detector = new OpenCVFaceDetector(0.5);
      ImagePanel panel = null;
      Mat image = new Mat();
      MatOfByte mem = new MatOfByte();
      
      while (true)
      {
         cap.read(image);
         Imgcodecs.imencode(".bmp", image, mem);
         BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(mem.toArray()));
         detector.detectAndOutline(bufferedImage);
         if (panel == null)
         {
            panel = ShowImages.showWindow(bufferedImage, "faces");
         }
         else
         {
            panel.setBufferedImageSafe(bufferedImage);
         }
      }
   }
}
