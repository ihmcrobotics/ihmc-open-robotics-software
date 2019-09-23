package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import javax.imageio.ImageIO;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

public class LidarImageFusionDataLoader
{
   public static BufferedImage readImage(File imageFile)
   {
      if (!imageFile.canRead())
      {
         LogTools.warn("No imageFile");
         return null;
      }
      
      try
      {
         return ImageIO.read(imageFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return null;
   }

   public static BufferedImage readImage(String path)
   {
      File file = new File(path);

      return readImage(file);
   }

   public static Point3D[] readPointCloud(File dataFile)
   {
      if (!dataFile.canRead())
      {
         LogTools.warn("No dataFile");
         return null;
      }

      int maximumNumberOfPoints = 200000;
      Point3D[] pointCloudBuffer = new Point3D[maximumNumberOfPoints];
      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(dataFile));
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
            Integer.parseInt(idxyzcolorArray[0]);
            double x = Double.parseDouble(idxyzcolorArray[1]);
            double y = Double.parseDouble(idxyzcolorArray[2]);
            double z = Double.parseDouble(idxyzcolorArray[3]);
            Integer.parseInt(idxyzcolorArray[4]);

            pointCloudBuffer[lineIndex] = new Point3D(x, y, z);
            lineIndex++;
         }
      }

      Point3D[] resizedPointCloud = new Point3D[lineIndex];
      for (int i = 0; i < resizedPointCloud.length; i++)
      {
         resizedPointCloud[i] = new Point3D(pointCloudBuffer[i]);
      }

      return resizedPointCloud;
   }

   public static Point3D[] readPointCloud(String path)
   {
      File pointCloudDataFile = new File(path);

      return readPointCloud(pointCloudDataFile);
   }
}