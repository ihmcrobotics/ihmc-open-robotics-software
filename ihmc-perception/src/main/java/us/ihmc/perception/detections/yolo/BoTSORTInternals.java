package us.ihmc.perception.detections.yolo;

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
   private static final int dimX = 8;
   private static final int dimZ = 4;

   private static final float STD_WEIGHT_POSITION = 1.0f / 20.0f;
   private static final float STD_WEIGHT_VELOCITY = 1.0f / 160.0f;

   private final float[][] F = new float[dimX][dimX];
   private final float[][] H = new float[dimZ][dimX];

   private final float[] mean = new float[dimX];
   private final float[][] cov = new float[dimX][dimX];

   public KalmanFilter()
   {
      for (int i = 0; i < dimX; i++)
         F[i][i] = 1f;
      for (int i = 0; i < 4; i++)
         F[i][i + 4] = 1f;

      for (int i = 0; i < dimZ; i++)
         H[i][i] = 1f;

      for (int i = 0; i < dimX; i++)
         cov[i][i] = 1f;
   }

   public void initiate(float[] z)
   {
      mean[0] = z[0];
      mean[1] = z[1];
      mean[2] = z[2];
      mean[3] = z[3];
      for (int i = 4; i < dimX; i++)
         mean[i] = 0f;

      float w = Math.max(1f, z[2]);
      float h = Math.max(1f, z[3]);

      float[] std = new float[8];
      std[0] = 2f  * STD_WEIGHT_POSITION * w;
      std[1] = 2f  * STD_WEIGHT_POSITION * h;
      std[2] = 2f  * STD_WEIGHT_POSITION * w;
      std[3] = 2f  * STD_WEIGHT_POSITION * h;
      std[4] = 10f * STD_WEIGHT_VELOCITY * w;
      std[5] = 10f * STD_WEIGHT_VELOCITY * h;
      std[6] = 10f * STD_WEIGHT_VELOCITY * w;
      std[7] = 10f * STD_WEIGHT_VELOCITY * h;

      zero(cov);
      for (int i = 0; i < dimX; i++)
         cov[i][i] = std[i] * std[i];
   }

   public void predict()
   {
      float[] newMean = mul(F, mean);
      System.arraycopy(newMean, 0, mean, 0, dimX);

      float w = Math.max(1f, mean[2]);
      float h = Math.max(1f, mean[3]);

      float[] stdPos = new float[] {
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h,
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h
      };
      float[] stdVel = new float[] {
            STD_WEIGHT_VELOCITY * w,
            STD_WEIGHT_VELOCITY * h,
            STD_WEIGHT_VELOCITY * w,
            STD_WEIGHT_VELOCITY * h
      };

      float[][] Q = new float[dimX][dimX];
      Q[0][0] = stdPos[0] * stdPos[0];
      Q[1][1] = stdPos[1] * stdPos[1];
      Q[2][2] = stdPos[2] * stdPos[2];
      Q[3][3] = stdPos[3] * stdPos[3];
      Q[4][4] = stdVel[0] * stdVel[0];
      Q[5][5] = stdVel[1] * stdVel[1];
      Q[6][6] = stdVel[2] * stdVel[2];
      Q[7][7] = stdVel[3] * stdVel[3];

      float[][] FCFt = mul(mul(F, cov), transpose(F));
      addInPlace(FCFt, Q, cov);
   }

   public void update(float[] z)
   {
      float[] projectedMean = mul(H, mean);
      float[][] projectedCov = mul(mul(H, cov), transpose(H));

      float w = Math.max(1f, mean[2]);
      float h = Math.max(1f, mean[3]);

      float[] std = new float[] {
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h,
            STD_WEIGHT_POSITION * w,
            STD_WEIGHT_POSITION * h
      };

      float[][] innovationCov = new float[dimZ][dimZ];
      innovationCov[0][0] = std[0] * std[0];
      innovationCov[1][1] = std[1] * std[1];
      innovationCov[2][2] = std[2] * std[2];
      innovationCov[3][3] = std[3] * std[3];

      float[][] S = add(projectedCov, innovationCov);
      float[][] covHt = mul(cov, transpose(H));
      float[][] Sinv = inv4(S);
      float[][] K = mul(covHt, Sinv);

      float[] y = subVec(z, projectedMean);
      float[] Ky = mul(K, y);
      for (int i = 0; i < dimX; i++)
         mean[i] += Ky[i];

      float[][] KSKt = mul(mul(K, S), transpose(K));
      float[][] newCov = sub(cov, KSKt);
      copy(newCov, cov);
   }

   public float[] getMean()
   {
      float[] out = new float[dimX];
      System.arraycopy(mean, 0, out, 0, dimX);
      return out;
   }

   public void setXY(float cx, float cy)
   {
      mean[0] = cx;
      mean[1] = cy;
   }

   private static float[] subVec(float[] a, float[] b)
   {
      float[] r = new float[a.length];
      for (int i = 0; i < a.length; i++) r[i] = a[i] - b[i];
      return r;
   }

   private static float[][] transpose(float[][] A)
   {
      float[][] T = new float[A[0].length][A.length];
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            T[j][i] = A[i][j];
      return T;
   }

   private static float[][] add(float[][] A, float[][] B)
   {
      float[][] C = new float[A.length][A[0].length];
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            C[i][j] = A[i][j] + B[i][j];
      return C;
   }

   private static float[][] sub(float[][] A, float[][] B)
   {
      float[][] C = new float[A.length][A[0].length];
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            C[i][j] = A[i][j] - B[i][j];
      return C;
   }

   private static float[][] mul(float[][] A, float[][] B)
   {
      int n = A.length, m = B[0].length, k = B.length;
      float[][] C = new float[n][m];
      for (int i = 0; i < n; i++)
         for (int j = 0; j < m; j++)
         {
            float s = 0f;
            for (int t = 0; t < k; t++) s += A[i][t] * B[t][j];
            C[i][j] = s;
         }
      return C;
   }

   private static float[] mul(float[][] A, float[] x)
   {
      float[] y = new float[A.length];
      for (int i = 0; i < A.length; i++)
      {
         float s = 0f;
         for (int j = 0; j < x.length; j++) s += A[i][j] * x[j];
         y[i] = s;
      }
      return y;
   }

   private static void zero(float[][] A)
   {
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            A[i][j] = 0f;
   }

   private static void copy(float[][] src, float[][] dst)
   {
      for (int i = 0; i < src.length; i++)
         System.arraycopy(src[i], 0, dst[i], 0, src[i].length);
   }

   private static void addInPlace(float[][] A, float[][] B, float[][] dst)
   {
      for (int i = 0; i < A.length; i++)
         for (int j = 0; j < A[0].length; j++)
            dst[i][j] = A[i][j] + B[i][j];
   }

   private static float[][] inv4(float[][] A)
   {
      int n = 4;
      float[][] aug = new float[n][2 * n];
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
         aug[i][n + i] = 1f;
      }

      for (int col = 0; col < n; col++)
      {
         int pivotRow = col;
         float best = Math.abs(aug[col][col]);
         for (int r = col + 1; r < n; r++)
         {
            float v = Math.abs(aug[r][col]);
            if (v > best) { best = v; pivotRow = r; }
         }
         if (pivotRow != col)
         {
            float[] tmp = aug[col];
            aug[col] = aug[pivotRow];
            aug[pivotRow] = tmp;
         }

         float pivot = aug[col][col];
         if (Math.abs(pivot) < 1e-9f)
            pivot = (pivot >= 0f ? 1e-9f : -1e-9f);

         for (int j = 0; j < 2 * n; j++) aug[col][j] /= pivot;

         for (int r = 0; r < n; r++)
         {
            if (r == col) continue;
            float f = aug[r][col];
            if (f == 0f) continue;
            for (int j = 0; j < 2 * n; j++) aug[r][j] -= f * aug[col][j];
         }
      }

      float[][] inv = new float[n][n];
      for (int i = 0; i < n; i++)
         System.arraycopy(aug[i], n, inv[i], 0, n);
      return inv;
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