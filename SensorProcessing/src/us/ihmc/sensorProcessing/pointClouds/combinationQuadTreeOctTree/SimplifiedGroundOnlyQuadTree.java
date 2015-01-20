package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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
   private final double noiseAmplitude = 0.0; //0.02;
   
   public SimplifiedGroundOnlyQuadTree(double minX, double minY, double maxX, double maxY, double resolution, double heightThreshold, double maxMultiLevelZChangeToFilterNoise)
   {
      super(minX, minY, maxX, maxY, resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise);
   }

   @Override
   public double heightAtPoint(double x, double y)
   {
      return get(x, y);
   }

   @Override
   public boolean addPoint(double x, double y, double z)
   {
      double noisyX = x + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyY = y + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyZ = z + RandomTools.generateRandomDouble(random, noiseAmplitude);
      
      CleanQuadTreePutResult result = put(noisyX, noisyY, noisyZ);
      return result.treeChanged;
   }
   
   @Override
   public boolean addToQuadtree(double x, double y, double z)
   {
      double noisyX = x + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyY = y + RandomTools.generateRandomDouble(random, noiseAmplitude);
      double noisyZ = z + RandomTools.generateRandomDouble(random, noiseAmplitude);
      
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
   
}
