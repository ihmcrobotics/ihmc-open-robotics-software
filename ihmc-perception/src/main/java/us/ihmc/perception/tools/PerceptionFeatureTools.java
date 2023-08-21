package us.ihmc.perception.tools;

import org.bytedeco.opencv.opencv_core.DMatchVector;
import org.bytedeco.opencv.opencv_core.KeyPointVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_features2d.DescriptorMatcher;
import org.bytedeco.opencv.opencv_features2d.ORB;
import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;

public class PerceptionFeatureTools
{
   public void findCorrespondences(Mat previousImage, Mat currentImage, ArrayList<Point2D> previousPoints, ArrayList<Point2D> currentPoints)
   {
      ORB orb = ORB.create();
      orb.setMaxFeatures(400);

      KeyPointVector previousImageFeatures = new KeyPointVector();
      Mat previousImageDescriptors = new Mat();

      KeyPointVector currentImageFeatures = new KeyPointVector();
      Mat currentImageDescriptors = new Mat();

      DMatchVector matches = new DMatchVector();

      // Extract ORB features from previous image using OpenCV
      orb.detectAndCompute(previousImage, new Mat(), previousImageFeatures, previousImageDescriptors);

      // Extract ORB features from current image using OpenCV
      orb.detectAndCompute(currentImage, new Mat(), currentImageFeatures, currentImageDescriptors);

      // Match features
      DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
      matcher.match(previousImageDescriptors, currentImageDescriptors, matches);

      // Fill in previousPoints and currentPoints
      for (int i = 0; i < matches.size(); i++)
      {
         int previousImageIndex = matches.get(i).queryIdx();
         int currentImageIndex = matches.get(i).trainIdx();

         Point2D previousPoint = new Point2D(previousImageFeatures.get(previousImageIndex).pt().x(), previousImageFeatures.get(previousImageIndex).pt().y());
         Point2D currentPoint = new Point2D(currentImageFeatures.get(currentImageIndex).pt().x(), currentImageFeatures.get(currentImageIndex).pt().y());

         previousPoints.add(previousPoint);
         currentPoints.add(currentPoint);
      }
   }
}
