package us.ihmc.atlas.calib;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.ConfigChessboard;
import boofcv.alg.geo.calibration.CalibrationObservation;
import boofcv.factory.calib.FactoryCalibrationTarget;
import boofcv.struct.image.GrayF32;
import org.ejml.data.DenseMatrix64F;

import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.gui.feature.VisualizeFeatures;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.IntrinsicParameters;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

/**
 * Process data collected inside the network manager for calibration robot join angles using the camera.  For each image in the sequence it computes the
 * 6-DOF location of the calibration target.
 *
 * @author Peter Abeles
 */
public class DetectChessboardInKinematicsData
{
   public static final boolean DELETE_BAD_DATA = true;

   public static final int boardWidth = 4;
   public static final int boardHeight = 5;

   public static void renderVector(Graphics2D g2, Se3_F64 targetToCamera, Vector3D_F64 vector, IntrinsicParameters intrinsic, Color color)
   {
      Point3D_F64 center = new Point3D_F64();

      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);
      Point2D_F64 centerPixel = PerspectiveOps.renderPixel(targetToCamera, K, new Point3D_F64());

      Vector3D_F64 a = vector.copy();
      a.scale(0.05);

      Point2D_F64 endPixel = PerspectiveOps.renderPixel(targetToCamera, K, center.plus(a));

      Vector2D_F64 v = new Vector2D_F64(endPixel.x - centerPixel.x, endPixel.y - centerPixel.y);
      v.normalize();
      v.x *= 50;
      v.y *= 50;

      g2.setColor(color);

      g2.setStroke(new BasicStroke(2));
      g2.drawLine((int) centerPixel.x, (int) centerPixel.y, (int) (centerPixel.x + v.x), (int) (centerPixel.y + v.y));
   }

   public static void renderOrientation(BufferedImage image, Se3_F64 targetToCamera, IntrinsicParameters intrinsic)
   {
      Vector3D_F64 axisX = new Vector3D_F64(1, 0, 0);
      Vector3D_F64 axisY = new Vector3D_F64(0, 1, 0);
      Vector3D_F64 axisZ = new Vector3D_F64(0, 0, 1);

      Graphics2D g2 = image.createGraphics();

      renderVector(g2, targetToCamera, axisX, intrinsic, Color.RED);
      renderVector(g2, targetToCamera, axisY, intrinsic, Color.WHITE);
      renderVector(g2, targetToCamera, axisZ, intrinsic, Color.BLUE);
   }

   public static void deleteDirectory(File dir)
   {
      if (!DELETE_BAD_DATA)
         return;

      File[] files = dir.listFiles();
      if (files == null)
      {
         System.out.println("Cannot list files in " + dir.getAbsolutePath());
         return;
      }

      for (File f : files)
      {
         if (!f.delete())
         {
            throw new RuntimeException("Can't delete file: " + f.getName());
         }
      }
      if (!dir.delete())
         System.out.println("Can't delete directory: " + dir.getName());
   }

   public static void main(String[] args) throws IOException
   {
//      File directory = new File("../DarpaRoboticsChallenge/data/calibration20131208");
      File directory = new File("../DarpaRoboticsChallenge/data/armCalibratoin20131209/calibration_right");

      if (!directory.isDirectory())
         throw new RuntimeException("Not directory");

      IntrinsicParameters intrinsic = UtilIO.loadXML("../DarpaRoboticsChallenge/data/calibration_images/intrinsic_ros.xml");
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);

      CalibrationDetectorChessboard target = FactoryCalibrationTarget.detectorChessboard(new ConfigChessboard(boardWidth, boardHeight, 0.03));

      // Computes the homography
      Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target.getLayout());
      // decomposes the homography
      Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();


      File files[] = directory.listFiles();
      if (files == null)
      {
         System.out.println("Cannot list files in " + directory);
         return;
      }

      List<File> fileList = new ArrayList<>();
      fileList.addAll(Arrays.asList(files));
      Collections.sort(fileList);

      for (File f : fileList)
      {
         if (!f.isDirectory())
            continue;

         System.out.print("Processing " + f.getName() + " ");

         BufferedImage orig = UtilImageIO.loadImage(new File(f, "leftEyeImage.png").getAbsolutePath());
         GrayF32 input = ConvertBufferedImage.convertFrom(orig, (GrayF32) null);

         if (orig.getWidth() != intrinsic.width || orig.getHeight() != intrinsic.height)
            throw new IllegalArgumentException("Unexpected image size " + orig.getWidth() + " " + orig.getHeight());

         File outputTarget = new File(f, "target.txt");
         if (outputTarget.exists())
            outputTarget.delete();

         outputTarget.createNewFile();

         // process the image and check for failure condition
         if (!target.process(input))
         {
            System.out.println("  Failed to detect target!");
            deleteDirectory(f);
            continue;
         }

         // Ordered observations of calibration points on the target
         CalibrationObservation calibrationObservation = target.getDetectedPoints();

         // Compute the homography
         if (!computeH.computeHomography(calibrationObservation))
            throw new RuntimeException("Can't compute homography");

         DenseMatrix64F H = computeH.getHomography();

         // compute camera pose from the homography matrix
         decomposeH.setCalibrationMatrix(K);
         Se3_F64 targetToCamera = decomposeH.decompose(H);

         Graphics2D g2 = orig.createGraphics();

         // compute pixel error
         double totalError = 0;

         for (int i = 0; i < target.getLayout().size(); i++)
         {
            Point2D_F64 p = target.getLayout().get(i);
            Point3D_F64 p3 = new Point3D_F64(p.x, p.y, 0);
            Point2D_F64 obsPixel = calibrationObservation.get(i).pixel;

            SePointOps_F64.transform(targetToCamera, p3, p3);
            Point2D_F64 expected = PerspectiveOps.convertNormToPixel(K, new Point2D_F64(p3.x / p3.z, p3.y / p3.z), null);

            totalError += expected.distance(obsPixel);
         }

         double averageError = totalError / target.getLayout().size();
         System.out.println(" Average pixel error = " + averageError);

         if (averageError > 0.8)
         {
            System.out.println("  Pixel error is too large.  Deleting image!");
            deleteDirectory(f);
            continue;
         }

         // save the results
         PrintStream out = new PrintStream(outputTarget);

         out.println("# target-to-camera Rotation matrix then Translation");
         for (int i = 0; i < 3; i++)
         {
            for (int j = 0; j < 3; j++)
            {
               out.printf("%f ", targetToCamera.getR().get(i, j));
            }
            out.println();
         }
         out.println();
         for (int i = 0; i < 3; i++)
         {
            out.printf("%f ", targetToCamera.getT().getIndex(i));
         }
         out.println();
         out.println();
         out.println("# List of detected calibration points in pixels");
         for (CalibrationObservation.Point point : calibrationObservation.points)
         {
            out.printf("%f %f%n", point.pixel.x, point.pixel.y);
         }
         out.close();


         // render and display the results
         for (int i = 0; i < calibrationObservation.size(); i++)
         {
            Point2D_F64 p = calibrationObservation.get(i).pixel;
            if (i == 0)
               VisualizeFeatures.drawPoint(g2, (int) p.x, (int) p.y, 3, Color.YELLOW);
            else
               VisualizeFeatures.drawPoint(g2, (int) p.x, (int) p.y, 3, Color.RED);
         }

         renderOrientation(orig, targetToCamera, intrinsic);

         UtilImageIO.saveImage(orig, f.getAbsolutePath() + "/detected.jpg");
//         if( num++ == 0 )
//            ShowImages.showWindow(orig,f.getName());
      }


   }
}
