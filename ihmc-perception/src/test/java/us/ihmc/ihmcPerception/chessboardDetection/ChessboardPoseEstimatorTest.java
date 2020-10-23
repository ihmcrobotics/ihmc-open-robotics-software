package us.ihmc.ihmcPerception.chessboardDetection;

import static us.ihmc.robotics.Assert.assertNotNull;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;

@Disabled
public class ChessboardPoseEstimatorTest
{
   static final boolean DEBUG = false;

   @Test
   public void testOpenCVRegression4x7() throws IOException
   {

      final double gridWidth = 0.05;
      final int squareNumCol = 4;
      final int squareNumRow = 7;

      InputStream is99Failed = getClass().getClassLoader().getResourceAsStream("regression4x7/99_failed.png");

      OpenCVChessboardPoseEstimator openCVDetector = new OpenCVChessboardPoseEstimator(squareNumRow, squareNumCol, gridWidth);
      BufferedImage image = ImageIO.read(is99Failed);
      RigidBodyTransform transform = openCVDetector.detect(image, true);
      assertNotNull(transform);

   }

   @Test
   public void testOpenCVRegression4x5() throws IOException
         {

            final double gridWidth = 0.05;
            final int squareNumCol = 4;
            final int squareNumRow = 5;

            InputStream is20Failed = getClass().getClassLoader().getResourceAsStream("regression4x5/20_failed.png");
            InputStream is7Failed = getClass().getClassLoader().getResourceAsStream("regression4x5/7_failed.png");

            OpenCVChessboardPoseEstimator openCVDetector = new OpenCVChessboardPoseEstimator(squareNumRow, squareNumCol, gridWidth);
            for (InputStream f : new InputStream[] { is20Failed, is7Failed })
            {
               BufferedImage image = ImageIO.read(f);
               RigidBodyTransform transform = openCVDetector.detect(image, true);
               assertNotNull(transform);
      }
   }

   static BufferedImage cloneImage(BufferedImage bi)
   {
      ColorModel cm = bi.getColorModel();
      boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
      WritableRaster raster = bi.copyData(null);
      return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
   }

}
