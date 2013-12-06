package us.ihmc.darpaRoboticsChallenge.calib;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.calib.PlanarCalibrationDetector;
import boofcv.abst.geo.RefinePnP;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.PlanarCalibrationTarget;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.gui.feature.VisualizeFeatures;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.ImageFloat32;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.ejml.data.DenseMatrix64F;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Process data collected inside the network manager for calibration robot join angles using the camera.  For each image in the sequence it computes the
 * 6-DOF location of the calibration target.
 *
 * @author Peter Abeles
 */
public class DetectChessboardInKinematicsData
{
   public static final int boardWidth = 4;
   public static final int boardHeight = 5;

   public static void renderVector( Graphics2D g2 ,  Se3_F64 targetToCamera ,Vector3D_F64 vector , IntrinsicParameters intrinsic , Color color ) {
      Point3D_F64 center = new Point3D_F64();

      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);
      Point2D_F64 centerPixel = PerspectiveOps.renderPixel(targetToCamera,K,new Point3D_F64());

      Vector3D_F64 a = vector.copy();
      a.scale(0.05);

      Point2D_F64 endPixel = PerspectiveOps.renderPixel(targetToCamera,K,center.plus(a));

      Vector2D_F64 v = new Vector2D_F64(endPixel.x-centerPixel.x,endPixel.y-centerPixel.y);
      v.normalize();
      v.x *= 50;
      v.y *= 50;

      g2.setColor(color);

      g2.setStroke(new BasicStroke(2));
      g2.drawLine((int)centerPixel.x,(int)centerPixel.y,(int)(centerPixel.x+v.x),(int)(centerPixel.y+v.y));
   }

   public static void renderOrientation( BufferedImage image , Se3_F64 targetToCamera , IntrinsicParameters intrinsic) {
      Vector3D_F64 axisX = new Vector3D_F64(1,0,0);
      Vector3D_F64 axisY = new Vector3D_F64(0,1,0);
      Vector3D_F64 axisZ = new Vector3D_F64(0,0,1);

      Graphics2D g2 = image.createGraphics();

      renderVector(g2,targetToCamera,axisX,intrinsic,Color.RED);
      renderVector(g2,targetToCamera,axisY,intrinsic,Color.WHITE);
      renderVector(g2,targetToCamera,axisZ,intrinsic,Color.BLUE);
   }

   public static void main(String[] args) throws IOException
   {
      File directory = new File("../DarpaRoboticsChallenge/data/chessboard_joints_20131204");

      if( !directory.isDirectory() )
         throw new RuntimeException("Not directory");

      IntrinsicParameters intrinsic = BoofMiscOps.loadXML("../DarpaRoboticsChallenge/data/calibration_images/intrinsic_ros.xml");
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);

      PlanarCalibrationTarget target = FactoryPlanarCalibrationTarget.gridChess(boardWidth, boardHeight, 0.03);

      // Computes the homography
      Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target);
      // decomposes the homography
      Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();

      // To select different types of detectors add or remove comments below
      PlanarCalibrationDetector detector = FactoryPlanarCalibrationTarget.detectorChessboard(new ConfigChessboard(boardWidth, boardHeight));

      File files[] = directory.listFiles();

      for( File f : files ) {
         if( !f.isDirectory() )
            continue;

         System.out.print("Processing "+f.getName()+" ");

         BufferedImage orig = UtilImageIO.loadImage(new File(f,"leftEyeImage.png").getAbsolutePath());
         ImageFloat32 input = ConvertBufferedImage.convertFrom(orig, (ImageFloat32) null);

         if( orig.getWidth() != intrinsic.width || orig.getHeight() != intrinsic.height )
            throw new IllegalArgumentException("Unexpected image size "+orig.getWidth()+" "+orig.getHeight());

         File output = new File(f,"target.txt");
         if( output.exists() )
            output.delete();

         output.createNewFile();

         // process the image and check for failure condition
         if( !detector.process(input) ) {
            System.err.println("Failed to detect target in "+f.getName());
            continue;
         }

         // Ordered observations of calibration points on the target
         List<Point2D_F64> observations = detector.getPoints();

         // Compute the homography
         if( !computeH.computeHomography(observations) )
            throw new RuntimeException("Can't compute homography");

         DenseMatrix64F H = computeH.getHomography();

         // compute camera pose from the homography matrix
         decomposeH.setCalibrationMatrix(K);
         Se3_F64 targetToCamera = decomposeH.decompose(H);

         Graphics2D g2 = orig.createGraphics();

         // compute pixel error
         double totalError = 0;

         for( int i = 0; i < target.points.size(); i++ ) {
            Point2D_F64 p = target.points.get(i);
            Point3D_F64 p3 = new Point3D_F64(p.x,p.y,0);
            Point2D_F64 obsPixel = observations.get(i);

            SePointOps_F64.transform(targetToCamera,p3,p3);
            Point2D_F64 expected = PerspectiveOps.convertNormToPixel(K,new Point2D_F64(p3.x/p3.z,p3.y/p3.z),null);

            totalError += expected.distance(obsPixel);
         }

         System.out.println(" Average pixel error = "+(totalError/target.points.size()));

         // save the results
         PrintStream out = new PrintStream(output);

         out.println("# target-to-camera Rotation matrix then Translation");
         for( int i = 0; i < 3; i++ ) {
            for( int j = 0; j < 3; j++ ) {
               out.printf("%f ",targetToCamera.getR().get(j,i));
            }
            out.println();
         }
         out.println();
         for( int i = 0; i < 3; i++ ) {
            out.printf("%f ",targetToCamera.getT().getIndex(i));
         }
         out.println();
         out.close();

         // render and display the results
         for( int i = 0; i < observations.size(); i++ ) {
            Point2D_F64 p = observations.get(i);
            if( i == 0 )
               VisualizeFeatures.drawPoint(g2, (int) p.x, (int) p.y, 3, Color.YELLOW);
            else
               VisualizeFeatures.drawPoint(g2, (int) p.x, (int) p.y, 3, Color.RED);
         }

         renderOrientation(orig,targetToCamera,intrinsic);

         UtilImageIO.saveImage(orig,f.getAbsolutePath() + "/detected.jpg");
//         if( num++ == 0 )
            ShowImages.showWindow(orig,f.getName());
      }


   }
}
