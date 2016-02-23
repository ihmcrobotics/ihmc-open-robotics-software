package us.ihmc.ihmcPerception.faceDetection;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import javax.imageio.ImageIO;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class NaiveFaceTracker
{
   private final double SHIFT_DELTA = 200.0;

   private final ArrayList<Face> trackedFaces = new ArrayList<>();
   private final Set<Face> unmatchedFaces = new HashSet<>();

   public ArrayList<Face> update(Rect[] faces)
   {
      unmatchedFaces.clear();

      for(int i = 0; i < trackedFaces.size(); i++)
      {
         int oldFaceX = trackedFaces.get(i).facialBorder.x;
         int oldFaceY = trackedFaces.get(i).facialBorder.y;
         boolean matched = false;

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
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      VideoCapture cap = new VideoCapture(1);
      OpenCVFaceDetector faceDetector = new OpenCVFaceDetector(0.5);
      NaiveFaceTracker faceTracker = new NaiveFaceTracker();
      ImagePanel panel = null;
      Mat image = new Mat();
      MatOfByte mem = new MatOfByte();
      while (true)
      {
         cap.read(image);
//         Imgcodecs.imdecode(buf, flags)
         Imgcodecs.imencode(".bmp", image, mem);
         BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(mem.toArray()));

         Graphics2D g2 = bufferedImage.createGraphics();

         Rect[] faces = faceDetector.detect(bufferedImage);
         ArrayList<Face> trackedFaces = faceTracker.update(faces);

         for (int i = 0; i < trackedFaces.size(); i++)
         {
            g2.setColor(trackedFaces.get(i).getColor());
            g2.drawRect(trackedFaces.get(i).facialBorder.x, trackedFaces.get(i).facialBorder.y, trackedFaces.get(i).facialBorder.width,
                  trackedFaces.get(i).facialBorder.height);
         }

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
