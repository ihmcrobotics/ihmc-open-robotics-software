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

public class NaiveFaceTracker
{
   private final double SHIFT_DELTA = 200.0;

   private final OpenCVFaceDetector faceDetector;
   private final ArrayList<Face> trackedFaces = new ArrayList<>();
   private Rect[] lastDetectedFaces = new Rect[0];

   public NaiveFaceTracker(double scale)
   {
      faceDetector = new OpenCVFaceDetector(scale);
   }

   public void detect(BufferedImage bufferedImage)
   {
      Rect[] faces = faceDetector.detect(bufferedImage);
      Graphics2D g2 = bufferedImage.createGraphics();

      for(int i = 0; i < trackedFaces.size(); i++)
      {
         int oldFaceX = trackedFaces.get(i).facialBorder.x;
         int oldFaceY = trackedFaces.get(i).facialBorder.y;
         boolean alreadyMatched = false;

         g2.setColor(getColor(i));
         g2.drawRect(oldFaceX, oldFaceY, trackedFaces.get(i).facialBorder.width, trackedFaces.get(i).facialBorder.height);

         for(int j = 0; j < faces.length; j++)
         {
            if(!alreadyMatched && faces[j] != null)
            {
               int newFaceX = faces[j].x;
               int newFaceY = faces[j].y;

               double faceShift = Math.sqrt(Math.pow(newFaceX - oldFaceX, 2) + Math.pow(newFaceY - oldFaceY, 2));

               if(faceShift < SHIFT_DELTA)
               {
                  g2.drawRect(newFaceX, newFaceY, faces[j].width, faces[j].height);
                  alreadyMatched = true;
                  trackedFaces.get(i).updateCoordinates(faces[j]);
                  faces[j] = null;
               }
            }
         }
      }

      for(int j = 0; j < faces.length; j++)
      {
         if(faces[j] != null)
            trackedFaces.add(new Face(System.currentTimeMillis(), faces[j]));
      }
   }

   private Color getColor(int i)
   {
      switch (i)
      {
      case 0: return Color.CYAN;
      case 1: return Color.YELLOW;
      case 2: return Color.GREEN;
      default: return Color.WHITE;
      }
   }

   public ArrayList<Face> getTrackedFaces()
   {
      return trackedFaces;
   }

   class Face
   {
      private final long id;
      public int persistenceCount = 0;
      public Rect facialBorder;

      public Face(long id, Rect facialBorder)
      {
         this.id = id;
         this.facialBorder = facialBorder;
      }

      public void updateCoordinates(Rect coordinates)
      {
         this.facialBorder.set(new double[]{coordinates.x, coordinates.y, coordinates.width, coordinates.height});
         persistenceCount++;
      }

      public int getPersistenceCount()
      {
         return persistenceCount;
      }
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
