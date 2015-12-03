package us.ihmc.ihmcPerception.faceDetection;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import org.opencv.core.*;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import us.ihmc.ihmcPerception.chessboardDetection.OpenCVChessboardPoseEstimator;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.file.Paths;

public class OpenCVFaceDetector
{
   private final boolean DEBUG = true;

   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", "opencv_java2411");
   }
   CascadeClassifier faceDetector = new CascadeClassifier(Paths.get("..", "IHMCPerception", "resources", "faceDetection", "haarcascade_frontalface_alt.xml").toString());
   MatOfRect faces = new MatOfRect();
   Mat image= new Mat();
   final double scaleFactor;
   
   /**
    * @param scaleFactor scaleDownImage for faster processing (scaleFactor <1.0)
    */

   public OpenCVFaceDetector(double scaleFactor)
   {
      this.scaleFactor=scaleFactor;
   }

   public Rect[] detect(BufferedImage bufferedImage)
   {
      Mat image=OpenCVChessboardPoseEstimator.convertBufferedImageToMat(bufferedImage);
      Imgproc.resize(image, image, new Size((int)(image.width()*scaleFactor), (int)(image.height()*scaleFactor)));
      faceDetector.detectMultiScale(image, faces);
      Rect[] faceArray = faces.toArray();
      for(Rect face:faceArray)
      {
         face.x=(int)(face.x/scaleFactor);
         face.y=(int)(face.y/scaleFactor);
         face.width=(int)(face.width/scaleFactor);
         face.height=(int)(face.height/scaleFactor);
      }
      return faceArray;
   }

   public Rect[] detectAndOutline(BufferedImage bufferedImage)
   {
      Rect[] faces = detect(bufferedImage);
      Graphics2D g2 = bufferedImage.createGraphics();

      for (int i = 0; i < faces.length; i++)
      {
         if(DEBUG)
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
         Highgui.imencode(".bmp", image, mem);
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
