package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.StringTokenizer;

import javax.vecmath.Point3d;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.dataStructures.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.quadTree.CleanQuadTreePutResult;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadTree;
import us.ihmc.utilities.math.geometry.InclusionFunction;

public class SimplifiedGroundOnlyQuadTree extends SimplifiedQuadTree implements QuadTreeHeightMapInterface
{
   private final Random random = new Random(1776L);
   private final double noiseAmplitude = 0.0; //0.0; //0.02;
   
   private Random random2 = new Random();
   
   private final boolean CREATE_FILE = false;
   private File pointListFile;
   private BufferedWriter bufferedWriter;
   
   public SimplifiedGroundOnlyQuadTree(double minX, double minY, double maxX, double maxY, double resolution, double heightThreshold, double maxMultiLevelZChangeToFilterNoise)
   {
      super(minX, minY, maxX, maxY, resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise);

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
   public double heightAtPoint(double x, double y)
   {
      Double doubleValue = get(x, y);
      if (doubleValue == null) return Double.NaN;
      return doubleValue;
   }

   @Override
   public boolean addPoint(double x, double y, double z)
   {
      double noisyX = x + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyY = y + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyZ = z + RandomTools.generateRandomDouble(random, noiseAmplitude);
      
      writeToFile(noisyX, noisyY, noisyZ);
      
      CleanQuadTreePutResult result = put(noisyX, noisyY, noisyZ);
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

      CleanQuadTreePutResult result = this.put(noisyX, noisyY, noisyZ);
      return result.treeChanged;
   }

   @Override
   public boolean containsPoint(double x, double y)
   {
      Double zValue = get(x, y);
      if ((zValue != null) && (Double.isNaN(zValue)))
         return true;

      return false;
   }

   @Override
   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent)
   {
      return super.getPointsAtGridResolution(xCenter, yCenter, xExtent, yExtent);
   }

   @Override
   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
           InclusionFunction<Point3d> maskFunctionAboutCenter)
   {
     ArrayList<Point3d> pointsAtGridResolution = super.getPointsAtGridResolution(xCenter, yCenter, xExtent, yExtent);
     ArrayList<Point3d> filteredList = new ArrayList<Point3d>();
     
     for (Point3d point : pointsAtGridResolution)
     {
        if (maskFunctionAboutCenter.isIncluded(point))
        {
           filteredList.add(point);
        }
     }

      return filteredList;
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
      File file = new File(filename);
      FileInputStream fileInputStream = new FileInputStream(file);
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(fileInputStream));
            
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      
      String inputString;
      int numPoints = 0;
      while(((inputString = bufferedReader.readLine()) != null) && (numPoints < maxNumberOfPoints))
      {
         Point3d point = parsePoint3d(inputString);
         if (point != null) 
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
      StringTokenizer tokenizer = new StringTokenizer(inputString, "(,)");
      String tokenX = tokenizer.nextToken();
      String tokenY = tokenizer.nextToken();
      String tokenZ = tokenizer.nextToken();
      
      double x = Double.parseDouble(tokenX);
      double y = Double.parseDouble(tokenY);
      double z = Double.parseDouble(tokenZ);

      Point3d point = new Point3d(x, y, z);
      return point;
   }

}
