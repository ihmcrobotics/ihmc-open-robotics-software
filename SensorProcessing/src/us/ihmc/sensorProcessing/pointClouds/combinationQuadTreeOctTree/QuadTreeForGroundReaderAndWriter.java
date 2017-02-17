package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Random;
import java.util.StringTokenizer;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.random.RandomTools;

public class QuadTreeForGroundReaderAndWriter
{
   private final Random random = new Random(1776L);
   private double noiseAmplitudeForReading = 0.0;

   private BufferedWriter bufferedWriter;

   public QuadTreeForGroundReaderAndWriter()
   {
      openFileForWriting();
   }

   public void setNoiseForReading(double noiseAmplitudeForReading)
   {
      this.noiseAmplitudeForReading = noiseAmplitudeForReading;
   }

   public void openFileForWriting()
   {
      System.out.println("Creating file to save points in");
      File pointListFile = new File("pointList" + random.nextLong());

      try
      {
         bufferedWriter = new BufferedWriter(new FileWriter(pointListFile));
      }
      catch (Exception e)
      {
         bufferedWriter = null;
      }
   }

   public void writePoint(double pointX, double pointY, double pointZ)
   {
      if (bufferedWriter != null)
      {
         try
         {
            bufferedWriter.write("(" + pointX + ", " + pointY + ", " + pointZ + ")\n");
            bufferedWriter.flush();
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   public QuadTreeForGroundHeightMap readQuadTreeForGroundHeightMap(String filename, int skipPoints, int maxNumberOfPoints, Box bounds, double maxZ, QuadTreeForGroundParameters quadTreeParameters) throws IOException
   {
      File file = new File(filename);
      FileInputStream fileInputStream = new FileInputStream(file);

      return readQuadTreeForGroundHeightMap(fileInputStream, skipPoints, maxNumberOfPoints, bounds, maxZ, quadTreeParameters);
   }

   public QuadTreeForGroundHeightMap readQuadTreeForGroundHeightMap(InputStream inputSteam, int skipPoints, int maxNumberOfPoints, Box bounds, double maxZ, QuadTreeForGroundParameters quadTreeParameters) throws IOException
   {
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputSteam));

      return readQuadTreeForGroundHeightMap(bufferedReader, skipPoints, maxNumberOfPoints, bounds, maxZ, quadTreeParameters);
   }

   public QuadTreeForGroundHeightMap readQuadTreeForGroundHeightMap(BufferedReader bufferedReader, int skipPoints, int maxNumberOfPoints, Box bounds, double maxZ,
           QuadTreeForGroundParameters quadTreeParameters)
           throws IOException
   {
      QuadTreeForGroundHeightMap quadTreeToReturn = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);

      ArrayList<Point3D> points = readPointsFromBufferedReader(bufferedReader, skipPoints, maxNumberOfPoints, bounds, maxZ);

      for (Point3D point : points)
      {
         quadTreeToReturn.addToQuadtree(point.getX(), point.getY(), point.getZ());
      }

      return quadTreeToReturn;
   }


   public ArrayList<Point3D> readPointsFromFile(String filename, int skipPoints, int maxNumberOfPoints, Box bounds, double maxZ) throws IOException
   {
      FileInputStream inputStream = new FileInputStream(filename);
      return readPointsFromInputStream(inputStream, skipPoints, maxNumberOfPoints, bounds, maxZ);
   }
   
   public ArrayList<Point3D> readPointsFromInputStream(InputStream inputStream, int skipPoints, int maxNumberOfPoints, Box bounds, double maxZ) throws IOException
   {
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream));
      return readPointsFromBufferedReader(bufferedReader, skipPoints, maxNumberOfPoints, bounds, maxZ);
   }


   public ArrayList<Point3D> readPointsFromBufferedReader(BufferedReader bufferedReader, int skipPoints, int maxNumberOfPoints, Box bounds, double maxZ) throws IOException
   {
      ArrayList<Point3D> points = new ArrayList<Point3D>();

      String inputString;
      int numPoints = 0;
      while (((inputString = bufferedReader.readLine()) != null) && (numPoints < maxNumberOfPoints))
      {
         Point3D point = parsePoint3d(inputString);
         if ((point != null) && (bounds.containsOrEquals(point.getX(), point.getY())) && (point.getZ() < maxZ))
         {
            if (noiseAmplitudeForReading > 0.0)
            {
               point.add(RandomTools.generateRandomPoint(random, noiseAmplitudeForReading, noiseAmplitudeForReading, noiseAmplitudeForReading));
            }

            points.add(point);
            numPoints++;
         }
      }

      bufferedReader.close();

      return points;
   }

   private static Point3D parsePoint3d(String inputString)
   {
      StringTokenizer tokenizer = new StringTokenizer(inputString, "(,) ");

      // StringTokenizer tokenizer = new StringTokenizer(inputString, " ");

      int numberOfTokens = tokenizer.countTokens();

      if (numberOfTokens == 5)
      {
         String index = tokenizer.nextToken();

         // System.out.println(index);
      }

      String tokenX = tokenizer.nextToken();
      String tokenY = tokenizer.nextToken();
      String tokenZ = tokenizer.nextToken();

      if (numberOfTokens == 5)
      {
         String intensity = tokenizer.nextToken();
      }

      double x = Double.parseDouble(tokenX);
      double y = Double.parseDouble(tokenY);
      double z = Double.parseDouble(tokenZ);

      Point3D point = new Point3D(x, y, z);

      return point;
   }

}
