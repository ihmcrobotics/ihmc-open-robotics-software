package us.ihmc.ihmcPerception.vision.shapes;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OpenCVColoredCirclularBlobDetector
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }

   public static void main(String[] args)
   {
      VideoCapture capture = new VideoCapture(0);

      if (!capture.isOpened())
      {
         System.err.println("Couldn't open camera!");
         System.exit(-1);
      }

      ImagePanel imagePanel = null;

      final Mat currentCameraFrameMat = new Mat();
      final Mat currentCameraFrameMedianBlurMat = new Mat();
      final Mat currentCameraFrameInHSVMat = new Mat();
      final Mat houghCirclesOutputMat = new Mat();
      final Mat greenMat = new Mat();

      while(true)
      {
         boolean camReadSuccess = capture.read(currentCameraFrameMat);

         Imgproc.medianBlur(currentCameraFrameMat, currentCameraFrameMedianBlurMat, 5);

         Imgproc.cvtColor(currentCameraFrameMedianBlurMat, currentCameraFrameInHSVMat, Imgproc.COLOR_BGR2HSV);

         Core.inRange(currentCameraFrameInHSVMat, new Scalar(78, 100, 100), new Scalar(83, 255, 255), greenMat);

         Imgproc.GaussianBlur(greenMat, greenMat, new Size(13, 13), 2, 2);

//         Imgproc.threshold(greenMat, greenMat, 0, 255, Imgproc.THRESH_OTSU);

         if(imagePanel == null)
         {
            imagePanel = ShowImages.showWindow(OpenCVTools.convertMatToBufferedImage(greenMat), "Regular Camera View");
         }

         findCirclesAndDisplay(greenMat, houghCirclesOutputMat, greenMat, imagePanel);
      }
   }


   private static void findCirclesAndDisplay(Mat grayscaleMatToSearch, Mat circlesMat, Mat matToOverlay, ImagePanel imagePanel)
   {
      Imgproc.HoughCircles(grayscaleMatToSearch, circlesMat, Imgproc.CV_HOUGH_GRADIENT, 1, grayscaleMatToSearch.rows() / 8, 100, 15, 0, 0);

      for(int i = 0; i < circlesMat.cols(); i++)
      {
         double[] vCircle = circlesMat.get(0, i);

         Point pt = new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
         int radius = (int)Math.round(vCircle[2]);

         Imgproc.circle(matToOverlay, pt, radius, new Scalar(160, 0, 0), 5);
      }

      imagePanel.setBufferedImage(OpenCVTools.convertMatToBufferedImage(matToOverlay));
   }
}
