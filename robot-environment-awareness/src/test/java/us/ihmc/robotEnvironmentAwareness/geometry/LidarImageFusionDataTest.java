package us.ihmc.robotEnvironmentAwareness.geometry;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.bytedeco.javacv.Java2DFrameUtils;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_ximgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_ximgproc.SuperpixelSLIC;
import org.junit.jupiter.api.Test;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionDataFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionRawData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionRawDataLoader;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

public class LidarImageFusionDataTest
{
   private static final String pointCloudDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\stereovisionpointcloud.txt";
   private static final String labeledImageDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\labeledimage.txt";
   private static final String imageDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\image.jpg";

   private static final int imageWidth = 1024;
   private static final int imageHeight = 544;

   private final IntrinsicParameters intrinsicParameters = PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters;
   private final LidarImageFusionRawDataLoader dataLoader = new LidarImageFusionRawDataLoader();

   @Test
   public void dataInitializeTest()
   {
      String dataName = "dataOne";
      dataLoader.loadLidarImageFusionRawData(dataName, pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      LidarImageFusionRawData rawData = dataLoader.getRawData(dataName);
      long startTime = System.nanoTime();
      rawData.initializeSegments();
      LogTools.info("initializeSegments time is " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
   }

   @Test
   public void segmentSinglePropagationTest()
   {
      String dataName = "dataOne";
      dataLoader.loadLidarImageFusionRawData(dataName, pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      LidarImageFusionRawData rawData = dataLoader.getRawData(dataName);
      rawData.initializeSegments();
      LidarImageFusionDataFeatureUpdater updater = new LidarImageFusionDataFeatureUpdater(rawData);

      long startTime = System.nanoTime();
      updater.iterateSegmenataionPropagation(0);
      LogTools.info("propagating time is " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));

      updater.createSegmentNodeData(403, 1);
      updater.createSegmentNodeData(253, 2);
      updater.createSegmentNodeData(130, 3);
   }

   @Test
   public void dataConstructionFromJavaCV()
   {
      File imageFile = new File(imageDataFileName);
      BufferedImage image = null;
      try
      {
         image = ImageIO.read(imageFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      Mat imageMat = Java2DFrameUtils.toMat(image);
      int height = imageMat.rows();
      int width = imageMat.cols();

      System.out.println("height " + height);
      System.out.println("width " + width);

      long startConvertTime = System.nanoTime();
      Mat convertedMat = new Mat();
      opencv_imgproc.cvtColor(imageMat, convertedMat, opencv_imgproc.COLOR_RGB2HSV);
      show(convertedMat, image.getWidth(), image.getHeight());
      System.out.println("ConvertingTime " + Conversions.nanosecondsToSeconds(System.nanoTime() - startConvertTime));

      long startSuperpixelSLICTime = System.nanoTime();
      SuperpixelSLIC slic = opencv_ximgproc.createSuperpixelSLIC(convertedMat, opencv_ximgproc.SLIC, 30, 80);
      slic.iterate(6);
      slic.enforceLabelConnectivity(30);
      System.out.println("SuperpixelSLIC " + Conversions.nanosecondsToSeconds(System.nanoTime() - startSuperpixelSLICTime));

      Mat labeledImageMat = Java2DFrameUtils.toMat(image);
      slic.getLabelContourMask(labeledImageMat);
      show(labeledImageMat, image.getWidth(), image.getHeight());

      long startConvertTime2 = System.nanoTime();
      Mat convertedMat2 = new Mat();
      opencv_imgproc.cvtColor(imageMat, convertedMat2, opencv_imgproc.COLOR_RGB2HSV);
      show(convertedMat2, width, height);
      System.out.println("ConvertingTime2 " + Conversions.nanosecondsToSeconds(System.nanoTime() - startConvertTime2));

      long startSuperpixelSLICTime2 = System.nanoTime();
      SuperpixelSLIC slic2 = opencv_ximgproc.createSuperpixelSLIC(convertedMat2, opencv_ximgproc.SLIC, 30, 80);
      slic2.iterate(6);
      slic2.enforceLabelConnectivity(30);
      System.out.println("SuperpixelSLIC " + Conversions.nanosecondsToSeconds(System.nanoTime() - startSuperpixelSLICTime2));
   }

   @Test
   public void segmentationEndToEndTest()
   {
      File imageFile = new File(imageDataFileName);
      BufferedImage image = null;
      try
      {
         image = ImageIO.read(imageFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
      Mat imageMat = Java2DFrameUtils.toMat(image);
      
      long startConvertTime2 = System.nanoTime();
      Mat convertedMat2 = new Mat();
      opencv_imgproc.cvtColor(imageMat, convertedMat2, opencv_imgproc.COLOR_RGB2HSV);
      System.out.println("ConvertingTime2 " + Conversions.nanosecondsToSeconds(System.nanoTime() - startConvertTime2));

      long startSuperpixelSLICTime2 = System.nanoTime();
      SuperpixelSLIC slic2 = opencv_ximgproc.createSuperpixelSLIC(convertedMat2, opencv_ximgproc.SLIC, 30, 80);
      slic2.iterate(6);
      slic2.enforceLabelConnectivity(30);
      System.out.println("SuperpixelSLIC " + Conversions.nanosecondsToSeconds(System.nanoTime() - startSuperpixelSLICTime2));
   }
   
   @Test
   public void testMatBufferedImageConverters()
   {
      File imageFile = new File(imageDataFileName);
      BufferedImage image = null;
      try
      {
         image = ImageIO.read(imageFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      Mat imageMat = Java2DFrameUtils.toMat(image);
      show(imageMat, image.getWidth(), image.getHeight());
      show(Java2DFrameUtils.toBufferedImage(imageMat));
      
      System.out.println("done");
      while(true)
      {
         
      }
   }

   public static void show(Mat mat, int width, int height)
   {
      byte[] byteArray = new byte[width * height * 3];
      opencv_imgcodecs.imencode(".jpg", mat, byteArray);
      BufferedImage img = null;
      try
      {
         InputStream in = new ByteArrayInputStream(byteArray);
         img = ImageIO.read(in);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      JFrame frame = new JFrame();
      ImageIcon icon = new ImageIcon(img);
      JLabel label = new JLabel(icon);

      frame.add(label);
      frame.setVisible(true);
      frame.setSize(width, height);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }
   
   public static void show(BufferedImage image)
   {
      JFrame frame = new JFrame();
      ImageIcon icon = new ImageIcon(image);
      JLabel label = new JLabel(icon);

      frame.add(label);
      frame.setVisible(true);
      frame.setSize(image.getWidth(), image.getHeight());
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }
}
