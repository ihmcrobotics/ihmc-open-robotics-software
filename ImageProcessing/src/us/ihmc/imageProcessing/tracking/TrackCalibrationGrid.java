package us.ihmc.imageProcessing.tracking;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.calib.PlanarCalibrationDetector;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.ImageFloat32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;

/**
 * @author Peter Abeles
 */
public class TrackCalibrationGrid
{
   public static void main( String args[] ) {
      BufferedImage image = UtilImageIO.loadImage("/home/pja/Desktop/auto_exp/frame0002.jpg");
      ImageFloat32 gray = ConvertBufferedImage.convertFrom(image,(ImageFloat32)null);


      // Detects the target and calibration point inside the target

      PlanarCalibrationDetector target = FactoryPlanarCalibrationTarget.detectorChessboard(new ConfigChessboard(5, 6, 10));
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

      List<Point2D_F64> points = target.getDetectedPoints();

      Graphics2D g2 = image.createGraphics();

      int r = 2;
      int w = r*2+1;

      for( Point2D_F64 p : points ) {

         int x = (int)p.x-r;
         int y = (int)p.y-r;


         g2.fillOval(x,y,w,w);
      }

      ShowImages.showWindow(image,"Detected Features");

   }
}
