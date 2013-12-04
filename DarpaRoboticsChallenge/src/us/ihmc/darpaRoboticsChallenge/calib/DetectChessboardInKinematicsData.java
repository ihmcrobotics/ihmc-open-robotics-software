package us.ihmc.darpaRoboticsChallenge.calib;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.calib.PlanarCalibrationDetector;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.PlanarCalibrationTarget;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.gui.feature.VisualizeFeatures;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import org.ejml.data.DenseMatrix64F;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
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

         System.out.println("Processing "+f.getName());

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
         List<Point2D_F64> points = detector.getPoints();

         // Compute the homography
         if( !computeH.computeHomography(detector.getPoints()) )
            throw new RuntimeException("Can't compute homography");

         DenseMatrix64F H = computeH.getHomography();

         // compute camera pose from the homography matrix
         decomposeH.setCalibrationMatrix(K);
         Se3_F64 targetToCamera = decomposeH.decompose(H);

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
         Graphics2D g2 = orig.createGraphics();
         for( Point2D_F64 p : points )
            VisualizeFeatures.drawPoint(g2, (int) p.x, (int) p.y, 3, Color.RED);

         UtilImageIO.saveImage(orig,f.getAbsolutePath() + "/detected.jpg");

         File hack = new File(f,"detected.png");
         if( hack.exists())
            hack.delete();
      }


   }
}
