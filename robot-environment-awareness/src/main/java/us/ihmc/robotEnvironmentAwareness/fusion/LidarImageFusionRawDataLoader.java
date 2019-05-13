package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;

public class LidarImageFusionRawDataLoader
{
   private final Map<String, LidarImageFusionRawData> rawDataMap = new HashMap<String, LidarImageFusionRawData>();

   private static final int maximumNumberOfPoints = 200000;

   public LidarImageFusionRawDataLoader()
   {

   }

   private static Point3D[] loadPointCloudData(File pointCloudDataFile)
   {
      Point3D[] pointCloud = new Point3D[maximumNumberOfPoints];

      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(pointCloudDataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int lineIndex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] idxyzcolorArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched == null)
         {
            break;
         }
         else
         {
            idxyzcolorArray = lineJustFetched.split("\t");
            int id = Integer.parseInt(idxyzcolorArray[0]);
            double x = Double.parseDouble(idxyzcolorArray[1]);
            double y = Double.parseDouble(idxyzcolorArray[2]);
            double z = Double.parseDouble(idxyzcolorArray[3]);
            int rgb = Integer.parseInt(idxyzcolorArray[4]);

            pointCloud[lineIndex] = new Point3D(x, y, z);
            lineIndex++;
         }
      }

      Point3D[] resizedPointCloud = new Point3D[lineIndex];
      for (int i = 0; i < resizedPointCloud.length; i++)
      {
         resizedPointCloud[i] = pointCloud[i];
      }

      return resizedPointCloud;
   }

   public void loadLidarImageFusionRawData(String dataName, String pointCloudDataFileName, String labeledImageDataFileName, int imageWidth, int imageHeight,
                                           IntrinsicParameters intrinsicParameters)
   {
      File pointCloudDataFile = new File(pointCloudDataFileName);
      File labeledImageDataFile = new File(labeledImageDataFileName);

      Point3D[] pointCloud = loadPointCloudData(pointCloudDataFile);
      BufferedImage bufferedImage = new BufferedImage(imageWidth, imageHeight, BufferedImage.TYPE_INT_RGB);
      int[] labels = new int[imageWidth * imageHeight];

      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(labeledImageDataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int lineIndex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] rgblabelArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched == null)
         {
            break;
         }
         else
         {
            rgblabelArray = lineJustFetched.split("\t");
            int r = Integer.parseInt(rgblabelArray[0]);
            int g = Integer.parseInt(rgblabelArray[1]);
            int b = Integer.parseInt(rgblabelArray[2]);
            int label = Integer.parseInt(rgblabelArray[3]);

            Color color = new Color(r, g, b);
            int rgbColor = color.getRGB();
            bufferedImage.setRGB(lineIndex % imageWidth, lineIndex / imageWidth, rgbColor);
            labels[lineIndex] = label;
            lineIndex++;
         }
      }

      LidarImageFusionRawData rawData = new LidarImageFusionRawData(pointCloud, bufferedImage, labels, intrinsicParameters);
      rawDataMap.put(dataName, rawData);
   }

   public LidarImageFusionRawData getRawData(String dataName)
   {
      return rawDataMap.get(dataName);
   }
}
