package us.ihmc.robotEnvironmentAwareness.slam.tools;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import us.ihmc.euclid.tuple3D.Point3D;

public class PCDDataImporter
{
   private enum FieldType
   {
      XYZ, XYZ_NORMAL_CURVATURE;
   }

   public static Point3D[] getPointsFromFile(File dataFile)
   {
      if (!dataFile.canRead())
         new NullPointerException("No dataFile");

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

      boolean isAscii = false;
      FieldType fieldType = FieldType.XYZ;
      while (true)
      {
         String lineJustFetched = null;
         String[] infoArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
            System.out.println(lineJustFetched);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched.contains("FIELDS"))
         {
            infoArray = lineJustFetched.split(" ");
            System.out.println(infoArray.length);
            if (infoArray.length == 4)
            {
               if (infoArray[3].contains("z"))
                  fieldType = FieldType.XYZ;
            }
            else if (infoArray.length == 8)
            {
               if (infoArray[4].contains("normal") && infoArray[7].contains("curvature"))
                  fieldType = FieldType.XYZ_NORMAL_CURVATURE;
            }
            System.out.println("field type is " + fieldType.toString());
         }
         if (lineJustFetched.contains("DATA"))
         {
            if (lineJustFetched.contains("ascii"))
               isAscii = true;
            break;
         }
      }

      if (!isAscii)
      {
         System.out.println("The imported PCD file is not ascii format.");
         return null;
      }
      int numberOfPoints = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] xyzArray;
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
            if (lineJustFetched.contains("nan"))
               continue;
            xyzArray = lineJustFetched.split(" ");
            switch (fieldType)
            {
               case XYZ:
                  pointCloudBuffer[numberOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]),
                                                                 Double.parseDouble(xyzArray[1]),
                                                                 Double.parseDouble(xyzArray[2]));
                  break;
               case XYZ_NORMAL_CURVATURE:
                  pointCloudBuffer[numberOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]),
                                                                 Double.parseDouble(xyzArray[1]),
                                                                 Double.parseDouble(xyzArray[2]));
                  Double.parseDouble(xyzArray[3]);
                  Double.parseDouble(xyzArray[4]);
                  Double.parseDouble(xyzArray[5]);
                  Double.parseDouble(xyzArray[6]);
                  break;
            }

            numberOfPoints++;
         }
      }
      Point3D[] points = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         points[i] = pointCloudBuffer[i];
      }

      return points;
   }
}
