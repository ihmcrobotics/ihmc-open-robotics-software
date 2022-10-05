package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_features2d.DescriptorMatcher;
import org.bytedeco.opencv.opencv_features2d.ORB;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

import static org.bytedeco.opencv.global.opencv_features2d.drawKeypoints;
import static org.bytedeco.opencv.global.opencv_features2d.drawMatches;
import static org.bytedeco.opencv.global.opencv_highgui.*;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import static org.bytedeco.opencv.global.opencv_imgproc.*;

public class VisualOdometry
{

   private int appState = 1;
   private int code = -1;
   private boolean allow = true;

   private final ORB orb = ORB.create();

   private static final String DATASET_PATH = "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/";

   private Mat previousImage;
   private Mat currentImage;
   private Mat display;

   private final ArrayList<KeyPointTrack> tracks = new ArrayList<>();
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
      orb.setMaxFeatures(500);
   }

   public void update()
   {
      if (!initialized)
      {
         previousImage = loadImage(DATASET_PATH + fileName);
         previousKeypoints = extractKeypoints(previousImage, previousDescriptors);
         display = new Mat(previousImage.rows(), previousImage.cols(), previousImage.type());
         initialized = true;
         return;
      }

      if (allow)
      {
         currentImage = loadImage(DATASET_PATH + fileName);

         // Extract ORB feature keypoints
         currentKeypoints = extractKeypoints(currentImage, currentDescriptors);

         // Match keypoints to tracks

         matchKeypoints(previousDescriptors, currentDescriptors, matches);
         LogTools.info("Matches Found: {} {}", matches.size(), code);

         display = currentImage.clone();
         plotMatches(display, previousKeypoints, currentKeypoints, matches);

         //// Estimate relative camera pose
         //transform = estimateTransform(keypoints, previousKeypoints, matches);
         //
         //// Triangulate local map
         //points3d = triangulatePoints(keypoints, previousKeypoints, matches, transform);
         //
         //// Display tracks
         //appendKeypointsToTracks(keypoints);

         fileName = String.format("%1$6s", frameIndex).replace(' ', '0') + ".png";


         previousDescriptors = currentDescriptors.clone();
         previousImage = currentImage.clone();

         previousKeypoints.clear();
         previousKeypoints.put(currentKeypoints);


         allow = false;
      }

      code = displayImage(display);
      if (code == 32)
      {
         appState = (appState + 1) % 2;
         LogTools.info("Application State: {}", appState);
      }
      if (appState == 0)
      {
         allow = true;
         frameIndex++;
      }
      else if(appState == 1)
      {
         if(code == 65363)
         {
            allow = true;
            frameIndex++;
         }
      }


      //display(tracks, display);

   }

   public void addKeypointsToTracks(ArrayList<KeyPointTrack> tracks, KeyPointVector keypoints, Mat descriptors, DMatchVector matches)
   {
      if(matches == null)
      {

      }
   }

   public Mat loadImage(String filePath)
   {
      LogTools.info("File Loaded: {}", DATASET_PATH + fileName);
      return imread(DATASET_PATH + fileName);
   }

   public int displayImage(Mat image)
   {
      Mat largeImage = new Mat(image.rows() * 2, image.cols() * 2, image.type());
      resize(image, largeImage, new Size(image.cols() * 2, image.rows() * 2));
      imshow("Visual Odometry", largeImage);
      int code = waitKeyEx(30);
      if (code == 113)
         System.exit(0);

      return code;
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

      DMatchVectorVector knn_matches = new DMatchVectorVector();
      matcher.knnMatch(currentDesc, previousDesc, knn_matches, 2);

      //-- Filter matches using the Lowe's ratio test
      float ratio_thresh = 0.8f;
      for (int i = 0; i < knn_matches.size(); i++)
      {
         if (knn_matches.get(i).get(0).distance() < ratio_thresh * knn_matches.get(i).get(1).distance())
         {
            matchesToPack.push_back(knn_matches.get(i).get(0));
         }
      }

      return matchesToPack;
   }

   public void plotMatches(Mat image, KeyPointVector previousKp, KeyPointVector currentKp, DMatchVector matchesToPlot)
   {

      for (int i = 0; i < matchesToPlot.size(); i++)
      {
         circle(image, new Point((int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().x(), (int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().y()),
                3, new Scalar(255, 0, 0, 255), -1, 0, 0);

         circle(image, new Point((int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().x(), (int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().y()),
                3, new Scalar(0, 0, 255, 255), -1, 0, 0);

         line(image, new Point((int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().x(), (int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().y()),
              new Point((int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().x(), (int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().y()),
              new Scalar(155, 250, 250, 255), 2, 0, 0);
      }

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
