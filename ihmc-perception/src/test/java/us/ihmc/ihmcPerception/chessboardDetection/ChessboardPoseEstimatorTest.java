package us.ihmc.ihmcPerception.chessboardDetection;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class ChessboardPoseEstimatorTest
{
   static final boolean DEBUG = false;

   @ContinuousIntegrationTest(estimatedDuration = 2.6)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testSimpleAlmostFrontChessboard() throws IOException
   {

      final double gridWidth = 0.05;
      final int squareNumCol = 6;
      final int squareNumRow = 5;
      BufferedImage image = ImageIO.read(getClass().getClassLoader().getResourceAsStream("simple5x6Chessboard.jpg"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.2)
   @Test(timeout = 30000)
   public void testDrivingSimCheckerBoard() throws IOException
   {

      final double gridWidth = 0.0335;
      final int squareNumCol = 6;
      final int squareNumRow = 5;
      BufferedImage image = ImageIO.read(getClass().getClassLoader().getResourceAsStream("drivingSim2.png"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth, 1e-1);
   }

   //boofCV can't find checker board
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 5000)
   public void testImage3() throws IOException
   {

      final double gridWidth = 0.049;
      final int squareNumCol = 9;
      final int squareNumRow = 6;
      BufferedImage image = ImageIO.read(getClass().getClassLoader().getResourceAsStream("polarisHood7x4.jpg"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth, 1e-1);
   }

   //boofcv found checkerboard but pose estimated poorly
   @ContinuousIntegrationTest(estimatedDuration = 1.6)
   @Test(timeout = 30000)
   public void testImage1() throws IOException
   {

      final double gridWidth = 0.049;
      final int squareNumCol = 9;
      final int squareNumRow = 6;
      BufferedImage image = ImageIO.read(getClass().getClassLoader().getResourceAsStream("polarisHood9x6.jpg"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth, 1e-1);
   }

   static BufferedImage cloneImage(BufferedImage bi)
   {
      ColorModel cm = bi.getColorModel();
      boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
      WritableRaster raster = bi.copyData(null);
      return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
   }

   public void testSingleImage(BufferedImage image, int squareNumRow, int squareNumCol, double gridWidth, double comparisonEps) throws IOException
   {
      BoofCVChessboardPoseEstimator boofCVDetector = new BoofCVChessboardPoseEstimator(squareNumRow, squareNumCol, gridWidth);
      RigidBodyTransform boofCVTransform = boofCVDetector.detect(image);

      OpenCVChessboardPoseEstimator openCVDetector = new OpenCVChessboardPoseEstimator(squareNumRow, squareNumCol, gridWidth);
      RigidBodyTransform openCVTransform = openCVDetector.detect(image, false);

      if (DEBUG)
      {

         if (boofCVTransform != null)
         {
            BufferedImage tempImage = cloneImage(image);
            openCVDetector.drawAxis(tempImage, boofCVTransform, gridWidth * 3);
            openCVDetector.drawReprojectedPoints(tempImage, boofCVTransform, Color.RED);
            boofCVDetector.drawBox(tempImage, boofCVTransform, gridWidth * 2);
            ImageIO.write(tempImage, "png", new File("boofCVout.png"));
            System.out.println(boofCVTransform);
         }

         if (openCVTransform != null)
         {
            BufferedImage tempImage = cloneImage(image);
            openCVDetector.drawAxis(tempImage, openCVTransform, gridWidth * 3);
            openCVDetector.drawReprojectedPoints(tempImage, openCVTransform, Color.GREEN);
            boofCVDetector.drawBox(tempImage, openCVTransform, gridWidth * 2);
            ImageIO.write(tempImage, "png", new File("openCVout.png"));
            System.out.println(openCVTransform);
         }

      }

      if (boofCVTransform == null && openCVTransform != null)
      {
         fail("boofcv detector can't find chessboard, can't compare");
      }
      else if (openCVTransform == null && boofCVTransform != null)
      {
         fail("opencv detector can't find chessboard, can't compare");
      }
      else if (openCVTransform == null && boofCVTransform == null)
      {
         fail("both boofcv and opencv failed to find chessboard");
      }
      else
      {
         assertTrue(boofCVTransform.epsilonEquals(openCVTransform, comparisonEps));
      }

   }
}
