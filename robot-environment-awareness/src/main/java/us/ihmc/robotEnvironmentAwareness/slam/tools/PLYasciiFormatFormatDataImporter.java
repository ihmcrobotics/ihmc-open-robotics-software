package us.ihmc.robotEnvironmentAwareness.slam.tools;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import us.ihmc.euclid.tuple3D.Point3D;

public class PLYasciiFormatFormatDataImporter
{
   public static Point3D[] getPointsFromFile(File dataFile)
   {
      if (!dataFile.canRead())
         new NullPointerException("No dataFile");

      BufferedReader bufferedReader = null;
      try
      {
         System.out.println(dataFile.getAbsolutePath());
         bufferedReader = new BufferedReader(new FileReader(dataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int numberOfVertex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] infoArray = null;
         try
         {
            lineJustFetched = bufferedReader.readLine();
            System.out.println(lineJustFetched);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched.contains("format"))
         {
            if (!lineJustFetched.contains("ascii"))
               return null;
         }
         if (lineJustFetched.contains("element vertex"))
         {
            infoArray = lineJustFetched.split(" ");
            numberOfVertex = Integer.parseInt(infoArray[2]);
            System.out.println("number of vertex is " + numberOfVertex);
         }
         if (lineJustFetched.contains("end_header"))
         {
            break;
         }
      }
      Point3D[] pointCloudBuffer = new Point3D[numberOfVertex];
      
      int indexOfPoints = 0;
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
            if(indexOfPoints == numberOfVertex)
               break;
            xyzArray = lineJustFetched.split(" ");
            pointCloudBuffer[indexOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]), Double.parseDouble(xyzArray[1]), Double.parseDouble(xyzArray[2]));
            indexOfPoints++;
         }
      }
      Point3D[] points = new Point3D[indexOfPoints];
      for (int i = 0; i < indexOfPoints; i++)
      {
         points[i] = pointCloudBuffer[i];
      }

      return points;
   }
}
