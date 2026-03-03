package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_core.Point2f;
import org.bytedeco.opencv.opencv_core.Point2fVector;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.Mat;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.javacpp.indexer.UByteIndexer;

import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_imgproc.*;
import static org.bytedeco.opencv.global.opencv_video.*;
import static org.bytedeco.opencv.global.opencv_calib3d.*;

public class GMC
{
   public enum Method { sparseOptFlow, none }

   private final Method method;
   private final int downscale;

   // params from python
   private final int maxCorners = 1000;
   private final double qualityLevel = 0.01;
   private final double minDistance = 1.0;
   private final int blockSize = 3;
   private final boolean useHarrisDetector = false;
   private final double k = 0.04;

   private Mat prevFrameGray = null;   // CV_8U
   private Mat prevKeypoints = null;   // Nx1 CV_32FC2
   private boolean initializedFirstFrame = false;

   public GMC(Method method, int downscale)
   {
      this.method = method == null ? Method.sparseOptFlow : method;
      this.downscale = Math.max(1, downscale);
   }

   /** Returns 2x3 affine (CV_64F). */
   public Mat apply(Mat bgrFrame)
   {
      if (method == Method.none || bgrFrame == null || bgrFrame.empty())
         return eye23();

      return applySparseOptFlow(bgrFrame);
   }

   private Mat applySparseOptFlow(Mat rawBgr)
   {
      Mat H = eye23(); // CV_64F

      // gray
      Mat gray = new Mat();
      cvtColor(rawBgr, gray, COLOR_BGR2GRAY);

      // downscale
      Mat graySmall = gray;
      Mat grayTmp = null;
      if (downscale > 1)
      {
         grayTmp = new Mat();
         resize(gray, grayTmp, new Size(gray.cols() / downscale, gray.rows() / downscale));
         graySmall = grayTmp;
      }

      // keypoints (SAFE overload: no mask argument)
      Mat keypoints = new Mat();
      goodFeaturesToTrack(graySmall, keypoints, maxCorners, qualityLevel, minDistance);

      // first frame
      if (!initializedFirstFrame)
      {
         prevFrameGray = graySmall.clone();
         prevKeypoints = keypoints.clone();
         initializedFirstFrame = true;

         // cleanup
         keypoints.release();
         gray.release();
         if (grayTmp != null) grayTmp.release();

         return H; // identity
      }

      // If no points, update state and return identity
      if (prevKeypoints == null || prevKeypoints.empty() || keypoints.empty())
      {
         if (prevFrameGray != null) prevFrameGray.release();
         prevFrameGray = graySmall.clone();

         if (prevKeypoints != null) prevKeypoints.release();
         prevKeypoints = keypoints.clone();

         keypoints.release();
         gray.release();
         if (grayTmp != null) grayTmp.release();

         return H;
      }

      // Optical flow
      Mat nextPts = new Mat();
      Mat status = new Mat();
      Mat err = new Mat();
      calcOpticalFlowPyrLK(prevFrameGray, graySmall, prevKeypoints, nextPts, status, err);

      Point2fVector prevGood = new Point2fVector();
      Point2fVector currGood = new Point2fVector();

      UByteIndexer st = status.createIndexer();
      FloatIndexer pIdx = prevKeypoints.createIndexer();
      FloatIndexer nIdx = nextPts.createIndexer();

      long n = status.rows();
      for (int i = 0; i < n; i++)
      {
         int ok = st.get(i, 0) & 0xFF;
         if (ok != 0)
         {
            float px = pIdx.get(i, 0, 0);
            float py = pIdx.get(i, 0, 1);
            float cx = nIdx.get(i, 0, 0);
            float cy = nIdx.get(i, 0, 1);
            prevGood.push_back(new Point2f(px, py));
            currGood.push_back(new Point2f(cx, cy));
         }
      }

      st.release();
      pIdx.release();
      nIdx.release();

      int goodCount = (int) prevGood.size();
      if (goodCount >= 5)
      {
         Mat prevMat = new Mat(goodCount, 1, CV_32FC2);
         Mat currMat = new Mat(goodCount, 1, CV_32FC2);

         FloatIndexer prevOut = prevMat.createIndexer();
         FloatIndexer currOut = currMat.createIndexer();

         for (int i = 0; i < goodCount; i++)
         {
            Point2f p = prevGood.get(i);
            Point2f q = currGood.get(i);
            prevOut.put(i, 0, 0, p.x());
            prevOut.put(i, 0, 1, p.y());
            currOut.put(i, 0, 0, q.x());
            currOut.put(i, 0, 1, q.y());
         }

         prevOut.release();
         currOut.release();

         Mat inliers = new Mat();
         Mat Hsmall = estimateAffinePartial2D(prevMat, currMat, inliers, RANSAC, 3.0, 2000, 0.99, 10);

         if (Hsmall != null && !Hsmall.empty())
         {
            if (downscale > 1)
            {
               Hsmall.ptr(0, 2).putDouble(Hsmall.ptr(0, 2).getDouble() * downscale);
               Hsmall.ptr(1, 2).putDouble(Hsmall.ptr(1, 2).getDouble() * downscale);
            }

            H.release();
            H = Hsmall.clone();
         }

         if (Hsmall != null) Hsmall.release();
         inliers.release();
         prevMat.release();
         currMat.release();
      }

      // update state
      if (prevFrameGray != null) prevFrameGray.release();
      prevFrameGray = graySmall.clone();

      if (prevKeypoints != null) prevKeypoints.release();
      prevKeypoints = keypoints.clone();

      // cleanup
      keypoints.release();
      gray.release();
      if (grayTmp != null) grayTmp.release();
      nextPts.release();
      status.release();
      err.release();

      return H;
   }

   private static Mat eye23()
   {
      // Clone to ensure the returned Mat owns its header safely
      return Mat.eye(2, 3, CV_64F).asMat().clone();
   }

   public void reset()
   {
      initializedFirstFrame = false;
      if (prevFrameGray != null) { prevFrameGray.release(); prevFrameGray = null; }
      if (prevKeypoints != null) { prevKeypoints.release(); prevKeypoints = null; }
   }
}