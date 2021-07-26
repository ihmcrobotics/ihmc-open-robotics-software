package us.ihmc.perception;

import nu.pattern.OpenCV;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class OpenPnPOpenCVDemo
{
   public static void main(String[] args)
   {
      OpenCV.loadShared();

      Mat mat = Mat.eye(3, 3, CvType.CV_8UC1);
      System.out.println("mat = " + mat.dump());
   }
}
