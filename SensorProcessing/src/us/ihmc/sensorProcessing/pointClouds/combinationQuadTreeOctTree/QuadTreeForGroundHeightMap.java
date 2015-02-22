package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.utilities.dataStructures.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.quadTree.Box;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGround;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundParameters;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundPutResult;
import us.ihmc.utilities.math.geometry.InclusionFunction;

public class QuadTreeForGroundHeightMap extends QuadTreeForGround implements QuadTreeHeightMapInterface
{
   private QuadTreeForGroundReaderAndWriter readerAndWriter = null;
   
   public QuadTreeForGroundHeightMap(Box bounds, QuadTreeForGroundParameters quadTreeParameters)
   {
      super(bounds, quadTreeParameters);
   }
   
   public void setupTreeForGroundReaderAndWriter(QuadTreeForGroundReaderAndWriter readerAndWriter)
   {
      this.readerAndWriter = readerAndWriter;
   }

   @Override
   public boolean addPoint(double x, double y, double z)
   {
      if (readerAndWriter != null) readerAndWriter.writePoint(x, y, z);
      
      // Set the default height to the first point you see. 
      // TODO: Should probably change this later to allow the user to select a point somewhere and 
      // Set that to be the default height...
      if (super.isEmpty())
      {
         super.setDefaultHeightWhenNoPoints(z);
      }
      
      
      QuadTreeForGroundPutResult result = put(x, y, z);
      return result.treeChanged;
   }

   @Override
   public boolean addToQuadtree(double x, double y, double z)
   {
      return addPoint(x, y, z);
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

   @Override
   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
           InclusionFunction<Point3d> maskFunctionAboutCenter)
   {
      ArrayList<Point3d> pointsWithinBoundsToPack = new ArrayList<Point3d>();
      ArrayList<Point3d> filteredPoints = new ArrayList<Point3d>();

      Box bounds = new Box(xCenter - xExtent, yCenter - yExtent, xCenter + xExtent, yCenter + yExtent);
      super.getAllPointsWithinBounds(bounds, pointsWithinBoundsToPack);
      maskList(pointsWithinBoundsToPack, maskFunctionAboutCenter, filteredPoints);

      // TODO: Magic number 10. Get rid of it somehow...
      if (filteredPoints.size() > 10)
         return filteredPoints;

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

      else
      {
         for (Point3d point : originalPoints)
         {
            if (maskFunctionAboutCenter.isIncluded(point))
            {
               maskedPointsToPack.add(point);
            }
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

   ArrayList<HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData>> hyperCubeTreeListeners = new ArrayList<HyperCubeTreeListener<GroundAirDescriptor,GroundOnlyQuadTreeData>>();
   
   @Override
   public void clearTree()
   {
      for (HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> listener : this.hyperCubeTreeListeners)
      {
         listener.treeCleared();
      }
      
      this.clear();
   }

   @Override
   public void addListener(HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> listener)
   {
      hyperCubeTreeListeners.add(listener);
   }

   @Override
   public void setUpdateQuadtree(boolean update)
   {
      // TODO Auto-generated method stub
   }

}
