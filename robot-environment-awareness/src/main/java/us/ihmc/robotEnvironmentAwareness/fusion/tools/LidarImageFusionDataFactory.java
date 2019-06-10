package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Random;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.bytedeco.javacpp.indexer.UByteRawIndexer;
import org.bytedeco.javacv.Java2DFrameUtils;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_ximgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_ximgproc.SuperpixelSLIC;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SegmentationRawData;

public class LidarImageFusionDataFactory
{
   private static final int MAX_NUMBER_OF_POINTS = 200000;
   private static final boolean displaySegmentedContour = false;

   public static LidarImageFusionData createLidarImageFusionData(Point3D[] pointCloud, BufferedImage bufferedImage, IntrinsicParameters intrinsicParameters,
                                                                 int pointCloudBufferSize, int pixelSize, double ruler, boolean enableConnectivity,
                                                                 int elementSize, int iterate)
   {
      ArrayList<SegmentationRawData> fusionDataSegments = new ArrayList<SegmentationRawData>();

      int imageWidth = bufferedImage.getWidth();
      int imageHeight = bufferedImage.getHeight();

      int[] labels = getNewLabels(bufferedImage, pixelSize, ruler, iterate, enableConnectivity, elementSize);

      if (labels.length != imageWidth * imageHeight)
         throw new RuntimeException("newLabels length is different with size of image " + labels.length + ", (w)" + imageWidth + ", (h)" + imageHeight);

      int numberOfLabels = 0;
      for (int i = 0; i < imageWidth * imageHeight; i++)
         if (numberOfLabels == labels[i])
            numberOfLabels++;

      for (int i = 0; i < numberOfLabels; i++)
         fusionDataSegments.add(new SegmentationRawData(i));

      Point3D[] pointCloudBuffer = randomPruningPointCloudData(pointCloud, pointCloudBufferSize);

      for (int i = 0; i < pointCloudBuffer.length; i++)
      {
         Point3D point = pointCloudBuffer[i];
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, intrinsicParameters);

         if (pixel[0] < 0 || pixel[0] >= bufferedImage.getWidth() || pixel[1] < 0 || pixel[1] >= bufferedImage.getHeight())
            continue;

         int arrayIndex = getArrayIndex(pixel[0], pixel[1], imageWidth);
         int label = labels[arrayIndex];

         fusionDataSegments.get(label).addPoint(new Point3D(point));
      }

      for (int u = 1; u < imageWidth - 1; u++)
      {
         for (int v = 1; v < imageHeight - 1; v++)
         {
            int curLabel = labels[getArrayIndex(u, v, imageWidth)];
            int[] labelsOfAdjacentPixels = new int[4];
            labelsOfAdjacentPixels[0] = labels[getArrayIndex(u, v - 1, imageWidth)]; // N
            labelsOfAdjacentPixels[1] = labels[getArrayIndex(u, v + 1, imageWidth)]; // S
            labelsOfAdjacentPixels[2] = labels[getArrayIndex(u - 1, v, imageWidth)]; // W
            labelsOfAdjacentPixels[3] = labels[getArrayIndex(u + 1, v, imageWidth)]; // E

            for (int labelOfAdjacentPixel : labelsOfAdjacentPixels)
            {
               if (curLabel != labelOfAdjacentPixel)
               {
                  if (!fusionDataSegments.get(curLabel).contains(labelOfAdjacentPixel))
                     fusionDataSegments.get(curLabel).addAdjacentSegmentLabel(labelOfAdjacentPixel);
               }
            }
         }
      }

      long startTime = System.nanoTime();
      for (SegmentationRawData fusionDataSegment : fusionDataSegments)
      {
         fusionDataSegment.update();
      }
      System.out.println("SegmentationRawData updating time " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));

      LidarImageFusionData newData = new LidarImageFusionData(fusionDataSegments);
      return newData;
   }

   /**
    * The type of the BufferedImage is TYPE_INT_RGB and the type of the Mat is CV_8UC3.
    */
   private static Mat convertBufferedImageToMat(BufferedImage bufferedImage)
   {
      Mat imageMat = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), opencv_core.CV_8UC3);
      int r, g, b;
      UByteRawIndexer indexer = imageMat.createIndexer();
      for (int y = 0; y < bufferedImage.getHeight(); y++)
      {
         for (int x = 0; x < bufferedImage.getWidth(); x++)
         {
            int rgb = bufferedImage.getRGB(x, y);

            r = (byte) ((rgb >> 0) & 0xFF);
            g = (byte) ((rgb >> 8) & 0xFF);
            b = (byte) ((rgb >> 16) & 0xFF);

            indexer.put(y, x, 0, r);
            indexer.put(y, x, 1, g);
            indexer.put(y, x, 2, b);
         }
      }
      indexer.release();

      return imageMat;
   }

   private static Point3D[] randomPruningPointCloudData(Point3D[] buffer, int desiredBufferSize)
   {
      int numberOfPoints = desiredBufferSize;
      if (desiredBufferSize > MAX_NUMBER_OF_POINTS)
         numberOfPoints = MAX_NUMBER_OF_POINTS;

      Point3D[] newBuffer = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         int randomIndex = new Random().nextInt(buffer.length);
         newBuffer[i] = buffer[randomIndex];
      }
      return newBuffer;
   }

   // TODO: Speed up.
   private static int[] getNewLabels(BufferedImage bufferedImage, int pixelSize, double ruler, int iterate, boolean enableConnectivity, int elementSize)
   {
      Mat imageMat = convertBufferedImageToMat(bufferedImage);
      Mat convertedMat = new Mat();
      opencv_imgproc.cvtColor(imageMat, convertedMat, opencv_imgproc.COLOR_RGB2HSV);
      SuperpixelSLIC slic = opencv_ximgproc.createSuperpixelSLIC(convertedMat, opencv_ximgproc.SLIC, pixelSize, (float) ruler);
      slic.iterate(iterate);
      if (enableConnectivity)
         slic.enforceLabelConnectivity(elementSize);

      Mat labelMat = new Mat();
      slic.getLabels(labelMat);

      int[] labels = new int[bufferedImage.getWidth() * bufferedImage.getHeight()];
      for (int i = 0; i < labels.length; i++)
      {
         labels[i] = labelMat.getIntBuffer().get(i);
      }

      if (displaySegmentedContour)
      {
         Mat contourMat = new Mat();
         slic.getLabelContourMask(contourMat);

         BufferedImage contourImage = Java2DFrameUtils.toBufferedImage(contourMat);
         BufferedImage copyOriginal = new BufferedImage(bufferedImage.getWidth(), bufferedImage.getHeight(), bufferedImage.getType());
         for (int i = 0; i < contourImage.getWidth(); i++)
         {
            for (int j = 0; j < contourImage.getHeight(); j++)
            {
               if (contourImage.getRGB(i, j) == -1)
                  copyOriginal.setRGB(i, j, 0xFF0000);
               else
                  copyOriginal.setRGB(i, j, bufferedImage.getRGB(i, j));
            }
         }
         show(copyOriginal);
      }

      return labels;
   }

   private static int getArrayIndex(int u, int v, int width)
   {
      return u + v * width;
   }

   private static void show(Mat mat, int width, int height)
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

   private static void show(BufferedImage image)
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
