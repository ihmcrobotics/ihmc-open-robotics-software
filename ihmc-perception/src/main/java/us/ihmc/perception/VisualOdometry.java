package us.ihmc.perception;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.indexer.IntIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_features2d.DescriptorMatcher;
import org.bytedeco.opencv.opencv_features2d.ORB;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

import static org.bytedeco.opencv.global.opencv_calib3d.*;
import static org.bytedeco.opencv.global.opencv_features2d.drawKeypoints;
import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import static org.bytedeco.opencv.global.opencv_imgproc.*;

// (TUM-RGBD) - fx:517.3 fy:516.5 cx:318.6 cy:255.3
// (KITTI) - fx:718.856 fy:718.856 cx:607.193 cy:185.216
// (IHMC-Chest-L515) - fx:602.259 fy:603.040 cx:321.375 cy:240.515

public class VisualOdometry
{

   private int appState = 0; // 0 is automatic, 1 is manual
   private int code = -1;
   private boolean allow = true;

   private final ORB orb = ORB.create();

      private static final String DATASET_PATH = "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/";
   //private static final String DATASET_PATH = "/home/bmishra/Workspace/Data/Datasets/dataset/sequences/00/image_2/";

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

   RigidBodyTransform transformToKf;

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

         matchKeypoints(previousKeypoints, previousDescriptors, currentKeypoints, currentDescriptors, matches);
         LogTools.info("Matches Found: {} {}", matches.size(), code);

         display = currentImage.clone();
         plotMatches(display, previousKeypoints, currentKeypoints, matches);

         Point2fVector previousPointsVector = new Point2fVector();
         Point2fVector currentPointsVector = new Point2fVector();
         for(DMatch match : matches.get())
         {
            previousPointsVector.put(new Point2f(previousKeypoints.get(match.trainIdx()).pt()));
            currentPointsVector.put(new Point2f(currentKeypoints.get(match.queryIdx()).pt()));
         }

         //// Estimate relative camera pose
         Mat mask = new Mat();
         RigidBodyTransform transform = estimateTransform(previousPointsVector, currentPointsVector, matches, mask);
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
      else if (appState == 1)
      {
         if (code == 65363)
         {
            allow = true;
            frameIndex++;
         }
      }

      //display(tracks, display);

   }

   public RigidBodyTransform estimateTransform(Point2fVector previousPoints, Point2fVector currentPoints, DMatchVector matches, Mat mask)
   {
      // (KITTI) -> fx:718.856 fy:718.856 cx:607.193 cy:185.216
      float data[] = {718.856f, 0.0f, 607.193f, 0.0f, 718.856f, 185.216f, 0.0f, 0.0f, 1.0f};
      Mat K = new Mat(3, 3, opencv_core.CV_32FC1, new FloatPointer(data));

      Mat previousPointsMat = new Mat(previousPoints);
      Mat currentPointsMat = new Mat(currentPoints);

      Mat R = new Mat(3, 3, opencv_core.CV_32FC1);
      Mat t = new Mat(1, 3, opencv_core.CV_32FC1);
      Mat E = findEssentialMat(previousPointsMat, currentPointsMat, K, RANSAC, 0.999, 1.0, mask);
      recoverPose(E, previousPointsMat, currentPointsMat, K, R, t, mask);

      IntIndexer indexer = mask.createIndexer();
      for (int i = (int)previousPoints.size() - 1; i >= 0; i--)
      {
         if ((int) indexer.get(i, 0) == 1)
         {
            previousPoints.erase(previousPoints.begin() + i);
            currentPoints.erase(currentPoints.begin() + i);
         }
      }
      LogTools.info("Inliers: {} {}", previousPoints.size(), currentKeypoints.size());

      Mat cvPose = new Mat(Mat.eye(4, 4, opencv_core.CV_32FC1));
      //R.copyTo(cvPose(Range(0, 3), Range(0, 3))); /* Excludes the 'end' element */
      //t.copyTo(cvPose(Range(0, 3), Range(3, 4)));
      //invert(cvPose, cvPose);
      return new RigidBodyTransform();
   }

   public void addKeypointsToTracks(ArrayList<KeyPointTrack> tracks, KeyPointVector keypoints, Mat descriptors, DMatchVector matches)
   {
      if (matches == null)
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
      resize(image, largeImage, new Size((int) (image.cols() * 1.5), (int) (image.rows() * 1.5)));
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

   public DMatchVector matchKeypoints(KeyPointVector previousKp, Mat previousDesc, KeyPointVector currentKp, Mat currentDesc, DMatchVector matchesToPack)
   {
      matchesToPack.clear();

      DMatchVectorVector knn_matches = new DMatchVectorVector();
      matcher.knnMatch(currentDesc, previousDesc, knn_matches, 2);

      //-- Filter matches using the Lowe's ratio test
      float ratio_thresh = 0.8f;
      for (int i = 0; i < knn_matches.size(); i++)
      {
         KeyPoint previousPoint = previousKp.get(knn_matches.get(i).get(0).trainIdx());
         KeyPoint currentPoint = currentKp.get(knn_matches.get(i).get(0).queryIdx());
         double distanceInPixels = EuclidGeometryTools.distanceBetweenPoint2Ds(previousPoint.pt().x(),
                                                                               previousPoint.pt().y(),
                                                                               currentPoint.pt().x(),
                                                                               currentPoint.pt().y());

         if (knn_matches.get(i).get(0).distance() < ratio_thresh * knn_matches.get(i).get(1).distance() && distanceInPixels < 100)
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
         circle(image,
                new Point((int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().x(), (int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().y()),
                3,
                new Scalar(255, 0, 0, 255),
                -1,
                0,
                0);

         circle(image,
                new Point((int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().x(), (int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().y()),
                3,
                new Scalar(0, 0, 255, 255),
                -1,
                0,
                0);

         line(image,
              new Point((int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().x(), (int) previousKp.get(matchesToPlot.get(i).trainIdx()).pt().y()),
              new Point((int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().x(), (int) currentKp.get(matchesToPlot.get(i).queryIdx()).pt().y()),
              new Scalar(155, 250, 250, 255),
              2,
              0,
              0);
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
