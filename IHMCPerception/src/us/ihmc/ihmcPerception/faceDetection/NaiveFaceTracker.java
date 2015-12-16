package us.ihmc.ihmcPerception.faceDetection;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Rect;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

public class NaiveFaceTracker
{
//   static
//   {
//      NativeLibraryLoader.loadLibrary("org.opencv", "opencv_java2411");
//   }

   private final double SHIFT_DELTA = 200.0;

   private final OpenCVFaceDetector faceDetector;
   private final ArrayList<Face> trackedFaces = new ArrayList<>();
   private final Set<Face> unmatchedFaces = new HashSet<>();

   public NaiveFaceTracker(double scale)
   {
      faceDetector = new OpenCVFaceDetector(scale);
   }

   public ArrayList<Face> detect(BufferedImage bufferedImage)
   {
      Rect[] faces = faceDetector.detect(bufferedImage);
      Graphics2D g2 = bufferedImage.createGraphics();

      unmatchedFaces.clear();

      for(int i = 0; i < trackedFaces.size(); i++)
      {
         int oldFaceX = trackedFaces.get(i).facialBorder.x;
         int oldFaceY = trackedFaces.get(i).facialBorder.y;
         boolean matched = false;

         g2.setColor(trackedFaces.get(i).getColor());
         g2.drawRect(oldFaceX, oldFaceY, trackedFaces.get(i).facialBorder.width, trackedFaces.get(i).facialBorder.height);

         for(int j = 0; j < faces.length; j++)
         {
            if(!matched && faces[j] != null)
            {
               int newFaceX = faces[j].x;
               int newFaceY = faces[j].y;

               double faceShift = Math.sqrt(Math.pow(newFaceX - oldFaceX, 2) + Math.pow(newFaceY - oldFaceY, 2));

               if(faceShift < SHIFT_DELTA)
               {
                  matched = true;
                  trackedFaces.get(i).updateCoordinates(faces[j]);
                  faces[j] = null;
               }
            }
         }

         if(!matched)
         {
            unmatchedFaces.add(trackedFaces.get(i));
         }
      }

      trackedFaces.removeAll(unmatchedFaces);

      for(int j = 0; j < faces.length; j++)
      {
         if(faces[j] != null)
            trackedFaces.add(new Face(System.nanoTime(), faces[j]));
      }

      return trackedFaces;
   }

   public static void main(String[] arg) throws IOException
   {
      NativeLibraryLoader.loadLibrary("org.opencv", "opencv_java2411");
      VideoCapture cap = new VideoCapture(0);
      NaiveFaceTracker faceTracker = new NaiveFaceTracker(0.5);
      ImagePanel panel = null;
      Mat image = new Mat();
      MatOfByte mem = new MatOfByte();
      while (true)
      {
         cap.read(image);
         Highgui.imencode(".bmp", image, mem);
         BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(mem.toArray()));
         faceTracker.detect(bufferedImage);
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
