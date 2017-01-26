package us.ihmc.imageProcessing.tracking;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.ConfigChessboard;
import boofcv.alg.geo.calibration.CalibrationObservation;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.factory.calib.FactoryCalibrationTarget;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import org.ejml.data.DenseMatrix64F;

import java.awt.*;
import java.awt.image.BufferedImage;

/**
 * @author Peter Abeles
 */
public class TrackCalibrationGrid
{
   public static void main( String args[] ) {
      BufferedImage image = UtilImageIO.loadImage("/home/pja/Desktop/auto_exp/frame0002.jpg");
      GrayF32 gray = ConvertBufferedImage.convertFrom(image, (GrayF32)null);


      // Detects the target and calibration point inside the target

      CalibrationDetectorChessboard target = FactoryCalibrationTarget.detectorChessboard(new ConfigChessboard(5, 6, 10));
      if( !target.process(gray) )
         throw new RuntimeException("Failed to detect target");


      Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target.getLayout());
      Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();

      if( !computeH.computeHomography(target.getDetectedPoints()) )
         throw new RuntimeException("Can't compute homography");

      DenseMatrix64F H = computeH.getHomography();

      decomposeH.setCalibrationMatrix(new DenseMatrix64F(3,3,true,new double[]{555,0,1024/2,0,555,544/2,0,0,1}));
      Se3_F64 motion = decomposeH.decompose(H);

      System.out.println("Translation: "+motion.getT());

      CalibrationObservation calibrationObservation = target.getDetectedPoints();

      Graphics2D g2 = image.createGraphics();

      int r = 2;
      int w = r*2+1;

      for( CalibrationObservation.Point point : calibrationObservation.points ) {

         Point2D_F64 p = point.pixel;
         int x = (int)p.x-r;
         int y = (int)p.y-r;


         g2.fillOval(x,y,w,w);
      }

      ShowImages.showWindow(image, "Detected Features");

   }
}
