package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.DMatchVector;
import org.bytedeco.opencv.opencv_core.KeyPointVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_features2d.DescriptorMatcher;
import org.bytedeco.opencv.opencv_features2d.ORB;
import us.ihmc.log.LogTools;

import static org.bytedeco.opencv.global.opencv_features2d.drawKeypoints;
import static org.bytedeco.opencv.global.opencv_features2d.drawMatches;
import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;

public class VisualOdometry
{
   private final ORB orb = ORB.create();

   private static final String DATASET_PATH = "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/";

   private Mat previousImage;
   private Mat currentImage;
   private Mat displayImage;

   private KeyPointVector currentKeypoints;
   private KeyPointVector previousKeypoints;

   final DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
   final DMatchVector matches = new DMatchVector();

   private Mat currentDescriptors = new Mat();
   private Mat previousDescriptors = new Mat();

   private String fileName = "000000.png";
   private int frameIndex = 0;
   private boolean initialized = false;


   public VisualOdometry()
   {
      orb.setMaxFeatures(400);
   }

   public void update()
   {
      if(!initialized)
      {
         previousImage = imread(DATASET_PATH + fileName);
         previousKeypoints = extractKeypoints(previousImage, previousDescriptors);
         displayImage = new Mat(previousImage.rows(), previousImage.cols(), previousImage.type());
         initialized = true;
         return;
      }

      currentImage = imread(DATASET_PATH + fileName);

      // Extract ORB feature keypoints
      currentKeypoints = extractKeypoints(currentImage, currentDescriptors);

      plotKeypoints(currentImage, displayImage, currentKeypoints);

      // Match previous keypoints
      matchKeypoints(previousDescriptors, currentDescriptors, matches);


      LogTools.info("Matches Found: {}", matches.size());
      for(int i = 0; i<matches.size(); i++)
      {
         LogTools.info("Match: {} -> {}", matches.get(i).trainIdx(), matches.get(i).queryIdx());
      }

      //
      //// Estimate relative camera pose
      //transform = estimateTransform(keypoints, previousKeypoints, matches);
      //
      //// Triangulate local map
      //points3d = triangulatePoints(keypoints, previousKeypoints, matches, transform);
      //
      //// Display tracks
      //appendKeypointsToTracks(keypoints);
      //display(tracks, displayImage);

      imshow("Visual Odometry", displayImage);
      int code = waitKeyEx(30);
      if(code == 113) System.exit(0);

      LogTools.info("File Name: {}", DATASET_PATH + fileName);

      fileName = String.format("%1$6s", frameIndex).replace(' ', '0') + ".png";
      frameIndex++;

      previousDescriptors = currentDescriptors.clone();
      previousImage = currentImage.clone();

      previousKeypoints.clear();
      previousKeypoints.put(currentKeypoints);


   }

   public void plotKeypoints(Mat image, Mat display, KeyPointVector keypoints)
   {
      drawKeypoints(image, keypoints, display);
   }

   public KeyPointVector extractKeypoints(Mat image, Mat descriptorsToPack)
   {
      KeyPointVector kp = new KeyPointVector();
      orb.detectAndCompute(image, new Mat(), kp, descriptorsToPack);
      return kp;
   }

   public DMatchVector matchKeypoints(Mat previousDesc, Mat currentDesc, DMatchVector matchesToPack)
   {
      matchesToPack.clear();
      matcher.match(previousDesc, currentDesc, matchesToPack);
      return matchesToPack;
   }

   public void plotMatches()
   {
      //final Mat links = new Mat();
      //final MatOfByte ones1 = new MatOfByte();
      //org.opencv.features2d.Features2d.drawMatches(draw, keypoints, draw, keypoints2, matches, links, new Scalar(0), new Scalar(0), ones1);
   }

   public void render()
   {

   }

   public static void main(String[] args)
   {
      VisualOdometry vo = new VisualOdometry();

      for (int i = 0; i < 4500; i++)
      {
         vo.update();
      }
   }
}
