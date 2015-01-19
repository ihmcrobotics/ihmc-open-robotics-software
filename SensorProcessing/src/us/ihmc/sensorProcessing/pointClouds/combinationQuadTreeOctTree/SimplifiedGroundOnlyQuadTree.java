package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.utilities.dataStructures.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.quadTree.CleanQuadTreePutResult;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadTree;
import us.ihmc.utilities.math.geometry.InclusionFunction;

public class SimplifiedGroundOnlyQuadTree extends SimplifiedQuadTree implements QuadTreeHeightMapInterface
{
   public SimplifiedGroundOnlyQuadTree(double minX, double minY, double maxX, double maxY, double resolution, double heightThreshold)
   {
      super(minX, minY, maxX, maxY, resolution, heightThreshold);
   }

   @Override
   public double heightAtPoint(double x, double y)
   {
      return get(x, y);
   }

   @Override
   public boolean addPoint(double x, double y, double z)
   {
      CleanQuadTreePutResult result = put(x, y, z);

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
      return this.getAllPointsWithinArea(xCenter, yCenter, xExtent, yExtent);
   }

   @Override
   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
           InclusionFunction<Point3d> maskFunctionAboutCenter)
   {
      return this.getAllPointsWithinArea(xCenter, yCenter, xExtent, yExtent, maskFunctionAboutCenter);
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
      // TODO Auto-generated method stub

   }

   @Override
   public boolean addToQuadtree(double x, double y, double z)
   {
	   CleanQuadTreePutResult result = this.put(x, y, z);
	   return result.treeChanged;
   }

  
   @Override
   public void setUpdateQuadtree(boolean update) 
   {
	   // TODO Auto-generated method stub

   }
}
