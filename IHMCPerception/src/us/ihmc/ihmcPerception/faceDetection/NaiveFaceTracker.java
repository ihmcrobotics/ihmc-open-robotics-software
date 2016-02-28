package us.ihmc.ihmcPerception.faceDetection;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import com.google.common.collect.HashMultiset;
import com.google.common.collect.Multiset;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.*;

public class NaiveFaceTracker
{
   private final double SHIFT_DELTA = 200.0;

   private final ArrayList<Rect> trackedFaces = new ArrayList<>();
   private final Set<Rect> unmatchedFaces = new HashSet<>();
   private final HashMultiset<Rect> newFaces = HashMultiset.create();

   public ArrayList<Rect> matchTrackedFaces(Rect[] faces)
   {
      unmatchedFaces.clear();

      for (int i = 0; i < faces.length; i++)
      {
         boolean matched = false;

         // compare each face to tracked faces
         for (int j = 0; j < trackedFaces.size(); j++)
         {
            if(!matched && isSameFace(faces[i], trackedFaces.get(j)))
            {
               matched = true;
               trackedFaces.get(j).set(new double[]{faces[i].x, faces[i].y, faces[i].width, faces[i].height});
               faces[i] = null;
            }
         }

         if(matched) continue;

         // compare each face to new faces that might have been false positives
         Iterator<Rect> iterator = newFaces.iterator();
         while(iterator.hasNext())
         {
            Rect newFace = iterator.next();
            if(!matched && isSameFace(faces[i], newFace))
            {
               matched = true;
               newFaces.add(newFace);
               newFace.set(new double[]{faces[i].x, faces[i].y, faces[i].width, faces[i].height});
               faces[i] = null;
            }

            if(newFaces.count(newFace) > 5)
            {
               trackedFaces.add(newFace);
               newFaces.remove(newFace, newFaces.count(newFace));
            }
         }

         if(matched) continue;

         // if still unmatched, add to newFaces
         if(faces[i] != null)
         {
            newFaces.add(faces[i]);
         }
      }

      trackedFaces.removeAll(unmatchedFaces);

      for(int j = 0; j < faces.length; j++)
      {
         if(faces[j] != null)
         {
            newFaces.add(faces[j]);
         }
      }

      return trackedFaces;
   }

   private boolean isSameFace(Rect newFace, Rect oldFace)
   {
      if(oldFace.equals(newFace)) return true;

      double newFaceX = newFace.x;
      double newFaceY = newFace.y;

      double oldFaceX = oldFace.x;
      double oldFaceY = oldFace.y;

      double faceShift = Math.sqrt(Math.pow(newFaceX - oldFaceX, 2) + Math.pow(newFaceY - oldFaceY, 2));

      return faceShift < SHIFT_DELTA;
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
         g2.setColor(Color.WHITE);

         Rect[] faces = faceDetector.detect(bufferedImage);
         ArrayList<Rect> trackedFaces = faceTracker.matchTrackedFaces(faces);

         for (int i = 0; i < trackedFaces.size(); i++)
         {
            g2.drawRect(trackedFaces.get(i).x, trackedFaces.get(i).y, trackedFaces.get(i).width,
                  trackedFaces.get(i).height);
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
