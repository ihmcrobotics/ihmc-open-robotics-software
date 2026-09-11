package us.ihmc.perception.detections.yolo;

import org.ejml.data.FMatrixRMaj;
import org.ejml.dense.row.CommonOps_FDRM;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.Point2f;
import org.bytedeco.opencv.opencv_core.Point2fVector;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.javacpp.indexer.UByteIndexer;

import static org.bytedeco.opencv.global.opencv_core.CV_32FC2;
import static org.bytedeco.opencv.global.opencv_core.CV_64F;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2GRAY;
import static org.bytedeco.opencv.global.opencv_imgproc.cvtColor;
import static org.bytedeco.opencv.global.opencv_imgproc.resize;
import static org.bytedeco.opencv.global.opencv_video.calcOpticalFlowPyrLK;
import static org.bytedeco.opencv.global.opencv_imgproc.goodFeaturesToTrack;
import static org.bytedeco.opencv.global.opencv_calib3d.RANSAC;
import static org.bytedeco.opencv.global.opencv_calib3d.estimateAffinePartial2D;

interface TrackableDetection
{
   String getObjectClass();

   double getConfidence();

   float getX1();
   float getY1();
   float getX2();
   float getY2();

   boolean has3D();
   float getCx();
   float getCy();
   float getCz();

   void setTrackId(int id);
   int getTrackId();
}

class KalmanFilter
{
   private static final int STATE_SIZE = 8;
   private static final int MEASUREMENT_SIZE = 4;
   private static final float STD_WEIGHT_POSITION = 1.0f / 20.0f;
   private static final float STD_WEIGHT_VELOCITY = 1.0f / 160.0f;

   private final FMatrixRMaj transition = CommonOps_FDRM.identity(STATE_SIZE);
   private final FMatrixRMaj observation = new FMatrixRMaj(MEASUREMENT_SIZE, STATE_SIZE);
   private final FMatrixRMaj mean = new FMatrixRMaj(STATE_SIZE, 1);
   private final FMatrixRMaj covariance = CommonOps_FDRM.identity(STATE_SIZE);

   private final FMatrixRMaj predictedMean = new FMatrixRMaj(STATE_SIZE, 1);
   private final FMatrixRMaj transitionCovariance = new FMatrixRMaj(STATE_SIZE, STATE_SIZE);
   private final FMatrixRMaj predictedCovariance = new FMatrixRMaj(STATE_SIZE, STATE_SIZE);
   private final FMatrixRMaj projectedMean = new FMatrixRMaj(MEASUREMENT_SIZE, 1);
   private final FMatrixRMaj observationCovariance = new FMatrixRMaj(MEASUREMENT_SIZE, STATE_SIZE);
   private final FMatrixRMaj innovationCovariance = new FMatrixRMaj(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
   private final FMatrixRMaj inverseInnovationCovariance = new FMatrixRMaj(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
   private final FMatrixRMaj covarianceObservationTranspose = new FMatrixRMaj(STATE_SIZE, MEASUREMENT_SIZE);
   private final FMatrixRMaj gain = new FMatrixRMaj(STATE_SIZE, MEASUREMENT_SIZE);
   private final FMatrixRMaj innovation = new FMatrixRMaj(MEASUREMENT_SIZE, 1);
   private final FMatrixRMaj gainInnovationCovariance = new FMatrixRMaj(STATE_SIZE, MEASUREMENT_SIZE);
   private final FMatrixRMaj covarianceCorrection = new FMatrixRMaj(STATE_SIZE, STATE_SIZE);

   public KalmanFilter()
   {
      for (int i = 0; i < MEASUREMENT_SIZE; i++)
         transition.set(i, i + MEASUREMENT_SIZE, 1.0f);
      CommonOps_FDRM.setIdentity(observation);
   }

   public void initiate(float[] z)
   {
      mean.zero();
      System.arraycopy(z, 0, mean.data, 0, MEASUREMENT_SIZE);
      covariance.zero();

      float width = Math.max(1.0f, z[2]);
      float height = Math.max(1.0f, z[3]);
      for (int i = 0; i < STATE_SIZE; i++)
      {
         float scale = i % 2 == 0 ? width : height;
         float standardDeviation = i < MEASUREMENT_SIZE ? 2.0f * STD_WEIGHT_POSITION * scale : 10.0f * STD_WEIGHT_VELOCITY * scale;
         covariance.set(i, i, standardDeviation * standardDeviation);
      }
   }

   public void predict()
   {
      // x = F x; P = F P F^T + Q.
      CommonOps_FDRM.mult(transition, mean, predictedMean);
      mean.set(predictedMean);
      CommonOps_FDRM.mult(transition, covariance, transitionCovariance);
      CommonOps_FDRM.multTransB(transitionCovariance, transition, predictedCovariance);
      covariance.set(predictedCovariance);

      float width = Math.max(1.0f, mean.get(2));
      float height = Math.max(1.0f, mean.get(3));
      for (int i = 0; i < STATE_SIZE; i++)
      {
         float scale = i % 2 == 0 ? width : height;
         float standardDeviation = (i < MEASUREMENT_SIZE ? STD_WEIGHT_POSITION : STD_WEIGHT_VELOCITY) * scale;
         covariance.add(i, i, standardDeviation * standardDeviation);
      }
   }

   public void update(float[] z)
   {
      // S = H P H^T + R; K = P H^T S^-1.
      CommonOps_FDRM.mult(observation, mean, projectedMean);
      CommonOps_FDRM.mult(observation, covariance, observationCovariance);
      CommonOps_FDRM.multTransB(observationCovariance, observation, innovationCovariance);

      float width = Math.max(1.0f, mean.get(2));
      float height = Math.max(1.0f, mean.get(3));
      for (int i = 0; i < MEASUREMENT_SIZE; i++)
      {
         float standardDeviation = STD_WEIGHT_POSITION * (i % 2 == 0 ? width : height);
         innovationCovariance.add(i, i, standardDeviation * standardDeviation);
      }

      if (!CommonOps_FDRM.invert(innovationCovariance, inverseInnovationCovariance))
         throw new IllegalStateException("Cannot invert Kalman filter innovation covariance");
      CommonOps_FDRM.multTransB(covariance, observation, covarianceObservationTranspose);
      CommonOps_FDRM.mult(covarianceObservationTranspose, inverseInnovationCovariance, gain);

      // x = x + K (z - H x); P = P - K S K^T.
      System.arraycopy(z, 0, innovation.data, 0, MEASUREMENT_SIZE);
      CommonOps_FDRM.subtractEquals(innovation, projectedMean);
      CommonOps_FDRM.multAdd(gain, innovation, mean);
      CommonOps_FDRM.mult(gain, innovationCovariance, gainInnovationCovariance);
      CommonOps_FDRM.multTransB(gainInnovationCovariance, gain, covarianceCorrection);
      CommonOps_FDRM.subtractEquals(covariance, covarianceCorrection);
   }

   public float[] getMean()
   {
      return mean.data.clone();
   }

   public void setXY(float cx, float cy)
   {
      mean.set(0, cx);
      mean.set(1, cy);
   }
}

class GMC
{
   public enum Method { sparseOptFlow, none }

   private final Method method;
   private final int downscale;

   private final int maxCorners = 1000;
   private final double qualityLevel = 0.01;
   private final double minDistance = 1.0;

   private Mat prevFrameGray = null;
   private Mat prevKeypoints = null;
   private boolean initializedFirstFrame = false;

   public GMC(Method method, int downscale)
   {
      this.method = method == null ? Method.sparseOptFlow : method;
      this.downscale = Math.max(1, downscale);
   }

   public Mat apply(Mat bgrFrame)
   {
      if (method == Method.none || bgrFrame == null || bgrFrame.empty())
         return eye23();

      return applySparseOptFlow(bgrFrame);
   }

   private Mat applySparseOptFlow(Mat rawBgr)
   {
      Mat H = eye23();

      Mat gray = new Mat();
      cvtColor(rawBgr, gray, COLOR_BGR2GRAY);

      Mat graySmall = gray;
      Mat grayTmp = null;
      if (downscale > 1)
      {
         grayTmp = new Mat();
         resize(gray, grayTmp, new Size(gray.cols() / downscale, gray.rows() / downscale));
         graySmall = grayTmp;
      }

      Mat keypoints = new Mat();
      goodFeaturesToTrack(graySmall, keypoints, maxCorners, qualityLevel, minDistance);

      if (!initializedFirstFrame)
      {
         prevFrameGray = graySmall.clone();
         prevKeypoints = keypoints.clone();
         initializedFirstFrame = true;

         keypoints.release();
         gray.release();
         if (grayTmp != null) grayTmp.release();

         return H;
      }

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

      if (prevFrameGray != null) prevFrameGray.release();
      prevFrameGray = graySmall.clone();

      if (prevKeypoints != null) prevKeypoints.release();
      prevKeypoints = keypoints.clone();

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
      return Mat.eye(2, 3, CV_64F).asMat().clone();
   }

   public void reset()
   {
      initializedFirstFrame = false;
      if (prevFrameGray != null) { prevFrameGray.release(); prevFrameGray = null; }
      if (prevKeypoints != null) { prevKeypoints.release(); prevKeypoints = null; }
   }
}