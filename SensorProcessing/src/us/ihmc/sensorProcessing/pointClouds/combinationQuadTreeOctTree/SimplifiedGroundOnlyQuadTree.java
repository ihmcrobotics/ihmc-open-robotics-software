package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.StringTokenizer;

import javax.vecmath.Point3d;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.dataStructures.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.quadTree.Box;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadTree;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadTreeParameters;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadTreePutResult;
import us.ihmc.utilities.math.geometry.InclusionFunction;

public class SimplifiedGroundOnlyQuadTree extends SimplifiedQuadTree implements QuadTreeHeightMapInterface
{
   private final Random random = new Random(1776L);
   private final double noiseAmplitude = 0.0; //0.0; //0.02;
   
   private Random random2 = new Random();
   
   private final boolean CREATE_FILE = false;
   private File pointListFile;
   private BufferedWriter bufferedWriter;
      
   public SimplifiedGroundOnlyQuadTree(Box bounds, SimplifiedQuadTreeParameters quadTreeParameters)
   {
      super(bounds, quadTreeParameters);
      
      if (CREATE_FILE)
      {
         System.out.println("Creating file to save points in");
         pointListFile = new File("pointList" + random2.nextLong());
         try
         {
            bufferedWriter = new BufferedWriter(new FileWriter(pointListFile));
         }
         catch(Exception e)
         {
            bufferedWriter = null;  
         }
      }
   }
   
   @Override
   public boolean addPoint(double x, double y, double z)
   {
      double noisyX = x + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyY = y + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyZ = z + RandomTools.generateRandomDouble(random, noiseAmplitude);
      
      writeToFile(noisyX, noisyY, noisyZ);
      
      SimplifiedQuadTreePutResult result = put(noisyX, noisyY, noisyZ);
      return result.treeChanged;
   }
   
   private void writeToFile(double pointX, double pointY, double pointZ)
   {
      if (bufferedWriter != null)
      {
         try
         {
            bufferedWriter.write("(" + pointX + ", " + pointY + ", " + pointZ + ")\n");
            bufferedWriter.flush();
         }
         catch(Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   @Override
   public boolean addToQuadtree(double x, double y, double z)
   {
      double noisyX = x + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyY = y + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyZ = z + RandomTools.generateRandomDouble(random, noiseAmplitude);
      
      writeToFile(noisyX, noisyY, noisyZ);

      SimplifiedQuadTreePutResult result = this.put(noisyX, noisyY, noisyZ);
      return result.treeChanged;
   }

   @Override
   public boolean containsPoint(double x, double y)
   {
      Double zValue = getHeightAtPoint(x, y);
      if ((zValue != null) && (Double.isNaN(zValue)))
         return true;

      return false;
   }

   @Override
   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent)
   {
      return getAllPointsWithinArea(xCenter, yCenter, xExtent, yExtent, null);
   }

//   @Override
//   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent, InclusionFunction<Point3d> maskFunctionAboutCenter)
//   {
//     ArrayList<Point3d> pointsAtGridResolution = super.getPointsAtGridResolution(xCenter, yCenter, xExtent, yExtent);
//     ArrayList<Point3d> filteredList = new ArrayList<Point3d>();
//     
//     for (Point3d point : pointsAtGridResolution)
//     {
//        if (maskFunctionAboutCenter.isIncluded(point))
//        {
//           filteredList.add(point);
//        }
//     }
//
//      return filteredList;
//   }
   
   
   @Override
   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent, InclusionFunction<Point3d> maskFunctionAboutCenter)
   {
//      System.out.println("\nxCenter = " + xCenter + " yCenter = " + yCenter + ", xExtent = " + xExtent + ", yExtent = " + yExtent);
      ArrayList<Point3d> pointsWithinBoundsToPack = new ArrayList<Point3d>();
      ArrayList<Point3d> filteredPoints = new ArrayList<Point3d>();

      Box bounds = new Box(xCenter-xExtent, yCenter-yExtent, xCenter+xExtent, yCenter+yExtent);
      super.getAllPointsWithinBounds(bounds, pointsWithinBoundsToPack);
      maskList(pointsWithinBoundsToPack, maskFunctionAboutCenter, filteredPoints);

      //TODO: Magic number 10. Get rid of it somehow...
      if (filteredPoints.size() > 10) return filteredPoints;
      
      // If not enough raw points, then use the heightAt function to do the best you can
      filteredPoints.clear();
      ArrayList<Point3d> pointsAtGridResolution = getPointsAtGridResolution(xCenter, yCenter, xExtent, yExtent);
      maskList(pointsAtGridResolution, maskFunctionAboutCenter, filteredPoints);

      return filteredPoints;
   }
   
   private ArrayList<Point3d> getPointsAtGridResolution(double centerX, double centerY, double extentX, double extentY)
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();

      for (double x = centerX - extentX; x <= centerX + extentX; x += getQuadTreeParameters().getResolution())
      {
         for (double y = centerY - extentY; y <= centerY + extentY; y += getQuadTreeParameters().getResolution())
         {
            double height = getHeightAtPoint(x, y);
            if (!Double.isNaN(height))
            {
               points.add(new Point3d(x, y, height));
            }
         }
      }
      return points;
   }

   private void maskList(ArrayList<Point3d> originalPoints, InclusionFunction<Point3d> maskFunctionAboutCenter, ArrayList<Point3d> maskedPointsToPack)
   {
      if (maskFunctionAboutCenter == null)
      {
         maskedPointsToPack.addAll(originalPoints);
      }
      
      for (Point3d point : originalPoints)
      {
         if (maskFunctionAboutCenter.isIncluded(point))
         {
            maskedPointsToPack.add(point);
         }
      }
   }
   
   @Override
   public void setOctree(Octree octree)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean addPointToOctree(double x, double y, double z)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void setUpdateOctree(boolean b)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void clearTree()
   {
      this.clear();
   }

   @Override
   public void addListener(HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> jmeGroundONlyQuadTreeVisualizer)
   {
//      this.listeners.add(jmeGroundONlyQuadTreeVisualizer);
   }
 

  
   @Override
   public void setUpdateQuadtree(boolean update) 
   {
	   // TODO Auto-generated method stub

   }
   
   public static ArrayList<Point3d> readPointsFromFile(String filename, int maxNumberOfPoints) throws IOException
   {
      return readPointsFromFile(filename, 0, maxNumberOfPoints, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   }
   
   public static ArrayList<Point3d> readPointsFromFile(String filename, int skipPoints, int maxNumberOfPoints, double minX, double minY, double maxX, double maxY, double maxZ) throws IOException
   {
      File file = new File(filename);
      FileInputStream fileInputStream = new FileInputStream(file);
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(fileInputStream));
            
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      
      String inputString;
      int numPoints = 0;
      while(((inputString = bufferedReader.readLine()) != null) && (numPoints < maxNumberOfPoints))
      {
         Point3d point = parsePoint3d(inputString);
         if ((point != null) && (point.getX() > minX) && (point.getY() > minY) && (point.getX() < maxX) && (point.getY() < maxY)&& (point.getZ() < maxZ)) 
         {
            points.add(point);
            numPoints++;
         }
      }
      bufferedReader.close();
      
      return points;
   }

   public static ArrayList<Point3d> readPointsFromResource(InputStream resourceName, int skipPoints, int maxNumberOfPoints, double minX, double minY, double maxX, double maxY, double maxZ) throws IOException
   {
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(resourceName));

      ArrayList<Point3d> points = new ArrayList<Point3d>();

      String inputString;
      int numPoints = 0;
      while(((inputString = bufferedReader.readLine()) != null) && (numPoints < maxNumberOfPoints))
      {
         Point3d point = parsePoint3d(inputString);
         if ((point != null) && (point.getX() > minX) && (point.getY() > minY) && (point.getX() < maxX) && (point.getY() < maxY)&& (point.getZ() < maxZ))
         {
            points.add(point);
            numPoints++;
         }
      }
      bufferedReader.close();

      return points;
   }
   
   private static Point3d parsePoint3d(String inputString)
   {
//      System.out.println(inputString);
      
      StringTokenizer tokenizer = new StringTokenizer(inputString, "(,)");
//      StringTokenizer tokenizer = new StringTokenizer(inputString, " ");
      
//      String index = tokenizer.nextToken();
//      System.out.println(index);

      String tokenX = tokenizer.nextToken();
//      System.out.println(tokenX);

      String tokenY = tokenizer.nextToken();
//      System.out.println(tokenY);

      String tokenZ = tokenizer.nextToken();
//      System.out.println(tokenZ);

      
//      String intensity = tokenizer.nextToken();
//      System.out.println(intensity);


      double x = Double.parseDouble(tokenX);
      double y = Double.parseDouble(tokenY);
      double z = Double.parseDouble(tokenZ);

      Point3d point = new Point3d(x, y, z);
      return point;
   }

}
