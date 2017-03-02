package us.ihmc.ihmcPerception.chessboardDetection;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class OpenCVChessboardPoseEstimator
{
   private static final boolean DEBUG = true;
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }

   private Size boardInnerCrossPattern;
   private double gridWidth;
   private Point3[] boardPoints;
   private MatOfPoint2f corners = new MatOfPoint2f();
   private TermCriteria termCriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.001);
   private Mat cameraMatrix = null;
   private MatOfDouble distCoeffs = new MatOfDouble();

   public OpenCVChessboardPoseEstimator(int rowsOfSquare, int colsOfSquare, double gridWidth)
   {
      setBoardSize(rowsOfSquare, colsOfSquare, gridWidth);
   }
   
   public void setMaxIterationsAndAccuracy(int maxIterations, double accuracyEpsilon)
   {
      termCriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, maxIterations, accuracyEpsilon);
   }

   public void setBoardSize(int rowsOfSquare, int colsOfSquare, double gridWidth)
   {
      boardInnerCrossPattern = new Size(colsOfSquare - 1, rowsOfSquare - 1);
      this.gridWidth = gridWidth;
      generateBoardPoints();
   }

   private void generateBoardPoints()
   {
      boardPoints = new Point3[(int) boardInnerCrossPattern.area()];
      int count = 0;
      for (int j = 0; j < boardInnerCrossPattern.height; j++)
      {
         for (int i = 0; i < boardInnerCrossPattern.width; i++)
         {
            double x = gridWidth * (i - boardInnerCrossPattern.width / 2 + 0.5);
            double y = -gridWidth * (j - boardInnerCrossPattern.height / 2 + 0.5);
            boardPoints[count++] = new Point3(x, y, 0.0);
         }
      }
   }

   public void setCameraMatrix(double fx, double fy, double cx, double cy)
   {
      cameraMatrix = Mat.zeros(3, 3, CvType.CV_32F);
      cameraMatrix.put(0, 0, fx); //fx
      cameraMatrix.put(1, 1, fy); //fy
      cameraMatrix.put(0, 2, cx); //cx
      cameraMatrix.put(1, 2, cy); //cy
      cameraMatrix.put(2, 2, 1);
   }

   public void setDefaultCameraMatrix(int rows, int cols, double fovY)
   {
      double f = rows / 2.0 / Math.tan(fovY / 2.0);
      cameraMatrix = Mat.zeros(3, 3, CvType.CV_32F);
      cameraMatrix.put(0, 0, f); //fx
      cameraMatrix.put(1, 1, f); //fy
      cameraMatrix.put(0, 2, cols / 2.0); //cx
      cameraMatrix.put(1, 2, rows / 2.0); //cy
      cameraMatrix.put(2, 2, 1);
   }
   
   public boolean findChessCornerRobustForExtremePitch(Mat original, Size patternSize, MatOfPoint2f corners, int flags, boolean attemptExtremePitchDetection)
   {
      boolean success = Calib3d.findChessboardCorners(original, boardInnerCrossPattern, corners, flags);
      if (success)
      {
         return success;
      }

      if(attemptExtremePitchDetection)
      {
         //try stretch image vertically
         Mat resizeMat = new Mat();
         Imgproc.resize(original, resizeMat, new Size(original.width(), original.height() * 2));
         if (Calib3d.findChessboardCorners(resizeMat, boardInnerCrossPattern, corners, flags))
         {
            Point[] cornerArray = corners.toArray();
            for (Point p : cornerArray)
               p.y /= 2;
            corners.fromArray(cornerArray);
            return true;
         }
      }

      return false;
   }

   public RigidBodyTransform detect(BufferedImage image, boolean attemptExtremePitchDetection)
   {
      if (cameraMatrix == null)
         setDefaultCameraMatrix(image.getHeight(), image.getWidth(), Math.PI / 4);
      Mat imageMat = OpenCVTools.convertBufferedImageToMat(image);
      int flags = Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_FAST_CHECK;
      if (findChessCornerRobustForExtremePitch(imageMat, boardInnerCrossPattern, corners, flags, attemptExtremePitchDetection))
      {
         Mat imageGray = new Mat(imageMat.rows(), imageMat.cols(), CvType.CV_8UC1);
         Imgproc.cvtColor(imageMat, imageGray, Imgproc.COLOR_BGRA2GRAY);
         Mat rvec = new Mat(), tvec = new Mat();

         int subPixSize = 3;
         Imgproc.cornerSubPix(imageGray, corners, new Size(subPixSize, subPixSize), new Size(-1, -1), termCriteria);
         //         Calib3d.drawChessboardCorners(imageGray, boardInnerCrossPattern, corners, true);

         Calib3d.solvePnPRansac(new MatOfPoint3f(boardPoints), corners, cameraMatrix, distCoeffs, rvec, tvec);

         RigidBodyTransform transform = opencvTRtoRigidBodyTransform(rvec, tvec);

         if (DEBUG)
         {
//            printMat(cameraMatrix);
            drawImagePoints(image, corners, Color.GREEN);
//            drawReprojectedPoints(image, rvec, tvec, Color.RED);
//            drawAxis(image, transform, 0.2);
         }
         return transform;
      }
      else
      {
         return null;
      }
   }

   public static RigidBodyTransform opencvTRtoRigidBodyTransform(Mat rvec, Mat tvec)
   {
      Vector3D translation = new Vector3D(tvec.get(0, 0)[0], tvec.get(1, 0)[0], tvec.get(2, 0)[0]);
      Vector3D axis = new Vector3D(rvec.get(0, 0)[0], rvec.get(1, 0)[0], rvec.get(2, 0)[0]);
      double angle = axis.length();
      axis.normalize();
      AxisAngle rotation = new AxisAngle(axis, angle);
      RigidBodyTransform transform = new RigidBodyTransform(rotation, translation);
      return transform;
   }

   public static void rigidBodyTransformToOpenCVTR(RigidBodyTransform transform, Mat tvec, Mat rvec)
   {
      Point3D translation = new Point3D();
      transform.getTranslation(translation);
      AxisAngle axisAngle = new AxisAngle();
      transform.getRotation(axisAngle);
      Vector3D rotVector = new Vector3D(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ());
      rotVector.normalize();
      rotVector.scale(axisAngle.getAngle());

      tvec.put(0, 0, translation.getX());
      tvec.put(1, 0, translation.getY());
      tvec.put(2, 0, translation.getZ());

      rvec.put(0, 0, rotVector.getX());
      rvec.put(1, 0, rotVector.getY());
      rvec.put(2, 0, rotVector.getZ());
   }

   public void drawAxis(BufferedImage image, RigidBodyTransform transform, double scale)
   {
      Mat tvec = new Mat(3, 1, CvType.CV_32F);
      Mat rvec = new Mat(3, 1, CvType.CV_32F);
      rigidBodyTransformToOpenCVTR(transform, tvec, rvec);
      drawAxis(image, rvec, tvec, scale);
   }

   public us.ihmc.euclid.tuple2D.Point2D getCheckerBoardImageOrigin(RigidBodyTransform transform)
   {
      Point3 origin = new Point3();
      Mat tvec = new Mat(3, 1, CvType.CV_32F);
      Mat rvec = new Mat(3, 1, CvType.CV_32F);
      rigidBodyTransformToOpenCVTR(transform, tvec, rvec);
      MatOfPoint2f imagePoints = new MatOfPoint2f();
      Calib3d.projectPoints(new MatOfPoint3f(origin), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
      return new us.ihmc.euclid.tuple2D.Point2D(imagePoints.get(0, 0));
   }

   private void drawAxis(BufferedImage image, Mat rvec, Mat tvec, double scale)
   {
      Point3 origin = new Point3();
      Point3 xAxis = new Point3(scale, 0.0, 0.0);
      Point3 yAxis = new Point3(0.0, scale, 0.0);
      Point3 zAxis = new Point3(0.0, 0.0, scale);

      MatOfPoint2f imagePoints = new MatOfPoint2f();
      Calib3d.projectPoints(new MatOfPoint3f(origin, xAxis), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
      drawImagePoints(image, imagePoints, Color.RED);
      Calib3d.projectPoints(new MatOfPoint3f(origin, yAxis), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
      drawImagePoints(image, imagePoints, Color.GREEN);
      Calib3d.projectPoints(new MatOfPoint3f(origin, zAxis), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
      drawImagePoints(image, imagePoints, Color.BLUE);
   }

   public static void printMat(Mat m)
   {
      System.out.println("Mat " + m.height() + "x" + m.width() + "\n------------------------------");
      for (int i = 0; i < m.height(); i++)
      {
         for (int j = 0; j < m.width(); j++)
         {
            System.out.print(String.format(" %8.2f", m.get(i, j)[0]));
         }
         System.out.println();
      }

      System.out.println("------------------------------");
   }

   public void drawReprojectedPoints(BufferedImage image, RigidBodyTransform transform, Color color)
   {
      Mat tvec = new Mat(3, 1, CvType.CV_32F);
      Mat rvec = new Mat(3, 1, CvType.CV_32F);
      rigidBodyTransformToOpenCVTR(transform, tvec, rvec);
      drawReprojectedPoints(image, rvec, tvec, color);
   }

   private void drawReprojectedPoints(BufferedImage image, Mat rvec, Mat tvec, Color color)
   {
      MatOfPoint2f imagePoints = new MatOfPoint2f();
      Calib3d.projectPoints(new MatOfPoint3f(boardPoints), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
      drawImagePoints(image, imagePoints, color);
   }

   public void drawImagePoints(BufferedImage image, Mat imagePoints, Color color)
   {
      Point2D[] point2dArray = new Point2D[imagePoints.rows()];
      for (int i = 0; i < point2dArray.length; i++)
      {
         double[] v = imagePoints.get(i, 0);
         point2dArray[i] = new Point2D(v[0], v[1]);
      }
      drawImagePoints(image, point2dArray, color);

   }

   public void drawImagePoints(BufferedImage image, Point2D[] imagePoints, Color color)
   {
      Graphics2D g2 = image.createGraphics();
      g2.setStroke(new BasicStroke(4));
      g2.setColor(color);

      for (int i = 0; i < imagePoints.length; i++)
      {
         if (i > 0)
         {
            int radius = 2;
            g2.drawOval((int) imagePoints[i].getX() - radius, (int) imagePoints[i].getY() - radius, 2 * radius, 2 * radius);
            g2.drawLine((int) imagePoints[i - 1].getX(), (int) imagePoints[i - 1].getY(), (int) imagePoints[i].getX(), (int) imagePoints[i].getY());
         }
         else
         {
            int radius = 12;
            g2.drawOval((int) imagePoints[i].getX() - radius, (int) imagePoints[i].getY() - radius, 2 * radius, 2 * radius);
         }

      }
      g2.finalize();
   }
}
