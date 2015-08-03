package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import javax.vecmath.Point3d;

import us.ihmc.robotics.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.quadTree.Box;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGround;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundParameters;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundPutResult;
import us.ihmc.robotics.geometry.InclusionFunction;

public class QuadTreeForGroundHeightMap extends QuadTreeForGround implements QuadTreeHeightMapInterface
{
   private final ReentrantLock lock = new ReentrantLock();
   
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
      lock();
      if (readerAndWriter != null) readerAndWriter.writePoint(x, y, z);
      
      // Set the default height to the first point you see if it were not set (ie NaN)
      if (super.isEmpty() && Double.isNaN(super.getDefaultHeightWhenNoPoints()))
      {
         super.setDefaultHeightWhenNoPoints(z);
      }
      
      
      QuadTreeForGroundPutResult result = put(x, y, z);
      
      unlock();
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
      double zValue = getHeightAtPoint(x, y);
      if ((Double.isNaN(zValue)))
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

      lock();
      Box bounds = new Box(xCenter - xExtent, yCenter - yExtent, xCenter + xExtent, yCenter + yExtent);
      super.getAllPointsWithinBounds(bounds, pointsWithinBoundsToPack);
      maskList(pointsWithinBoundsToPack, maskFunctionAboutCenter, filteredPoints);

      // TODO: Magic number 10. Get rid of it somehow...
      if (filteredPoints.size() > 10)
      {
         unlock();
         return filteredPoints;
      }

      // If not enough raw points, then use the heightAt function to do the best you can
      filteredPoints.clear();
      ArrayList<Point3d> pointsAtGridResolution = getPointsAtGridResolution(xCenter, yCenter, xExtent, yExtent);
      maskList(pointsAtGridResolution, maskFunctionAboutCenter, filteredPoints);
      unlock();
      return filteredPoints;
   }

   private ArrayList<Point3d> getPointsAtGridResolution(double centerX, double centerY, double extentX, double extentY)
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();

      lock();
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
      unlock();

      return points;
   }

   private static void maskList(ArrayList<Point3d> originalPoints, InclusionFunction<Point3d> maskFunctionAboutCenter, ArrayList<Point3d> maskedPointsToPack)
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

   ArrayList<HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData>> hyperCubeTreeListeners = new ArrayList<HyperCubeTreeListener<GroundAirDescriptor,GroundOnlyQuadTreeData>>();
   
   @Override
   public void clearTree(double defaultGroundHeight)
   {
      lock();
      for (HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> listener : this.hyperCubeTreeListeners)
      {
         listener.treeCleared();
      }
      
      this.clear();
      this.setDefaultHeightWhenNoPoints(defaultGroundHeight);
      unlock();
   }
   
   @Override
   public double getDefaultHeightWhenNoPoints()
   {
      return super.getDefaultHeightWhenNoPoints();
   }

   @Override
   public void addListener(HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> listener)
   {
      lock();
      hyperCubeTreeListeners.add(listener);
      unlock();
   }

   @Override
   public void lock()
   {
      lock.lock();
   }

   @Override
   public void unlock()
   {
      lock.unlock();
   }

   @Override
   public boolean hasPoints()
   {
      boolean hasPoints = false;
      lock.lock();
      hasPoints = getRootNode().hasChildren();
      lock.unlock();
      
      return hasPoints;
   }

}
