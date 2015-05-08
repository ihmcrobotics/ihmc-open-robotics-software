package us.ihmc.ihmcPerception.chessboardDetection;

import static org.junit.Assert.fail;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;

import static org.junit.Assert.*;

import javax.imageio.ImageIO;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.code.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;


public class ChessboardPoseEstimatorTest
{
   
   static final boolean DEBUG=true;
   @Test
   public void testSimpleAlmostFrontChessboard() throws IOException
   {

      final double gridWidth = 0.05;
      final int squareNumCol = 6;
      final int squareNumRow = 5;
      BufferedImage image = ImageIO.read(new File("testImages/simple5x6Chessboard.jpg"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth);
   }

   
   //boofCV can't find checker board
   @Ignore
   @Test
   public void testImage3() throws IOException
   {

      final double gridWidth = 0.049;
      final int squareNumCol = 9;
      final int squareNumRow = 6;
      BufferedImage image = ImageIO.read(new File("testImages/polarisHood7x4.jpg"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth);
   }

   //boofcv found checkerboard but pose estimated poorly
   @Test
   @Ignore
   public void testImage1() throws IOException
   {

      final double gridWidth = 0.049;
      final int squareNumCol = 9;
      final int squareNumRow = 6;
      BufferedImage image = ImageIO.read(new File("testImages/polarisHood9x6.jpg"));
      testSingleImage(image, squareNumRow, squareNumCol, gridWidth);
   }

   static BufferedImage cloneImage(BufferedImage bi)
   {
      ColorModel cm = bi.getColorModel();
      boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
      WritableRaster raster = bi.copyData(null);
      return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
   }
   

   public void testSingleImage(BufferedImage image, int squareNumRow, int squareNumCol, double gridWidth) throws IOException
   {
      BoofCVChessboardPoseEstimator boofCVDetector = new BoofCVChessboardPoseEstimator(squareNumRow, squareNumCol, gridWidth);
      RigidBodyTransform boofCVTransform = boofCVDetector.detect(image);

      OpenCVChessboardPoseEstimator openCVDetector = new OpenCVChessboardPoseEstimator(squareNumRow - 1, squareNumCol - 1, gridWidth);
      RigidBodyTransform openCVTransform = openCVDetector.detect(image);


      if (DEBUG)
      {

         if (boofCVTransform != null)
         {
            BufferedImage tempImage = cloneImage(image);
            openCVDetector.drawAxis(tempImage, boofCVTransform, gridWidth * 2);
            openCVDetector.drawReprojectedPoints(tempImage, boofCVTransform, Color.RED);
            boofCVDetector.drawBox(tempImage, boofCVTransform, gridWidth * 2);
            ImageIO.write(tempImage, "png", new File("boofCVout.png"));
            System.out.println(boofCVTransform);
         }

         if (openCVTransform != null)
         {
            BufferedImage tempImage = cloneImage(image);
            openCVDetector.drawAxis(tempImage, openCVTransform, gridWidth * 2);
            openCVDetector.drawReprojectedPoints(tempImage, openCVTransform, Color.GREEN);
            boofCVDetector.drawBox(tempImage, openCVTransform, gridWidth * 2);
            ImageIO.write(tempImage, "png", new File("openCVout.png"));
            System.out.println(openCVTransform);
         }

      }

      if(boofCVTransform!=null && openCVTransform!=null)
      {
         assertTrue(boofCVTransform.epsilonEquals(openCVTransform, 1e-3));
      }
      else
      {
         fail("one of the detector can't find chessboard, can't compare");
      }

   }

}
