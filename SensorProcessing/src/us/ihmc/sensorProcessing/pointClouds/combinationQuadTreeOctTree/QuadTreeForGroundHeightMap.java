package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.InclusionFunction;
import us.ihmc.robotics.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGround;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.quadTree.QuadTreeForGroundPutResult;

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
   public List<Point3D> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent)
   {
      return getAllPointsWithinArea(xCenter, yCenter, xExtent, yExtent, null);
   }

   @Override
   public List<Point3D> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
           InclusionFunction<Point3D> maskFunctionAboutCenter)
   {
      ArrayList<Point3D> pointsWithinBoundsToPack = new ArrayList<Point3D>();
      ArrayList<Point3D> filteredPoints = new ArrayList<Point3D>();

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
      ArrayList<Point3D> pointsAtGridResolution = getPointsAtGridResolution(xCenter, yCenter, xExtent, yExtent);
      maskList(pointsAtGridResolution, maskFunctionAboutCenter, filteredPoints);
      unlock();
      return filteredPoints;
   }

   private ArrayList<Point3D> getPointsAtGridResolution(double centerX, double centerY, double extentX, double extentY)
   {
      ArrayList<Point3D> points = new ArrayList<Point3D>();

      lock();
      for (double x = centerX - extentX; x <= centerX + extentX; x += getQuadTreeParameters().getResolution())
      {
         for (double y = centerY - extentY; y <= centerY + extentY; y += getQuadTreeParameters().getResolution())
         {
            double height = getHeightAtPoint(x, y);
            if (!Double.isNaN(height))
            {
               points.add(new Point3D(x, y, height));
            }
         }
      }
      unlock();

      return points;
   }

   private static void maskList(ArrayList<Point3D> originalPoints, InclusionFunction<Point3D> maskFunctionAboutCenter, ArrayList<Point3D> maskedPointsToPack)
   {
      if (maskFunctionAboutCenter == null)
      {
         maskedPointsToPack.addAll(originalPoints);
      }

      else
      {
         for (Point3D point : originalPoints)
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
