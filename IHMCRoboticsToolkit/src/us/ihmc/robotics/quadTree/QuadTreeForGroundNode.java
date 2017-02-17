package us.ihmc.robotics.quadTree;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.euclid.tuple3D.Point3D;

public class QuadTreeForGroundNode
{
   private final String id;
   private final Box bounds;
   private QuadTreeForGroundLeaf leaf = null;
   private double defaultHeightWhenNoPonts;
   
   private boolean hasChildren = false;
   private QuadTreeForGroundNode NW = null;
   private QuadTreeForGroundNode NE = null;
   private QuadTreeForGroundNode SE = null;
   private QuadTreeForGroundNode SW = null;

   private final QuadTreeForGroundPointLimiter pointLimiter;
   private final QuadTreeForGroundParameters parameters;
   private final ArrayList<QuadTreeForGroundListener> listeners;
   
   private final QuadTreeForGroundNode parent;

   public QuadTreeForGroundNode(String id, double minX, double minY, double maxX, double maxY, QuadTreeForGroundParameters parameters, QuadTreeForGroundPointLimiter decay, QuadTreeForGroundNode parent, double defaultHeightWhenNoPonts, 
                             ArrayList<QuadTreeForGroundListener> listeners)
   {
      this(id, new Box(minX, minY, maxX, maxY), parameters, decay, parent, defaultHeightWhenNoPonts, listeners);
   }

   public QuadTreeForGroundNode(String id, Box bounds, QuadTreeForGroundParameters quadTreeParameters, QuadTreeForGroundPointLimiter pointLimiter, QuadTreeForGroundNode parent, double defaultHeightWhenNoPonts, ArrayList<QuadTreeForGroundListener> listeners)
   {
      this.id = id;
      this.defaultHeightWhenNoPonts = defaultHeightWhenNoPonts;
      this.listeners = listeners;

      this.parameters = quadTreeParameters;
      this.pointLimiter = pointLimiter;

      this.bounds = bounds;

      for (QuadTreeForGroundListener listener : this.listeners)
      {
         if (!Double.isNaN(defaultHeightWhenNoPonts))
         {
            listener.nodeAdded(id, bounds, (float) (bounds.maxX - bounds.minX), (float) (bounds.maxY - bounds.minY), (float) defaultHeightWhenNoPonts);
         }
      }
      
      this.parent = parent;
   }

   public void setDefaultHeightWhenNoPoints(double defaultHeightWhenNoPonts)
   {
      this.defaultHeightWhenNoPonts = defaultHeightWhenNoPonts;      
   }
   
   public double getDefaultHeightWhenNoPoints()
   {
      return defaultHeightWhenNoPonts;
   }
   
   public void getAllSubTreePoints(Collection<Point3D> pointsToPack)
   {
      if (hasChildren)
      {
         NE.getAllSubTreePoints(pointsToPack);
         NW.getAllSubTreePoints(pointsToPack);
         SE.getAllSubTreePoints(pointsToPack);
         SW.getAllSubTreePoints(pointsToPack);
      }
      else if (leaf != null)
      {
         this.leaf.getAllPoints(pointsToPack);
      }
   }

   public void getCellAverageSubTreePoints(Collection<Point3D> pointsToPack)
   {
      if (hasChildren)
      {
         NE.getCellAverageSubTreePoints(pointsToPack);
         NW.getCellAverageSubTreePoints(pointsToPack);
         SE.getCellAverageSubTreePoints(pointsToPack);
         SW.getCellAverageSubTreePoints(pointsToPack);
      }
      else if (leaf != null)
      {
         pointsToPack.add(this.leaf.getAveragePoint());
      }
   }
   
   

   public void getAllDescendantNodes(ArrayList<QuadTreeForGroundNode> nodeListToPack)
   {
      if (hasChildren)
      {
         NE.getAllDescendantNodes(nodeListToPack);
         NW.getAllDescendantNodes(nodeListToPack);
         SE.getAllDescendantNodes(nodeListToPack);
         SW.getAllDescendantNodes(nodeListToPack);
      }
      else
      {
         nodeListToPack.add(this);
      }
   }

   /**
    * 
    * @param childrenToPack Childeren, in the order NW, NE, SE, SW
    */
   public void getChildrenNodes(ArrayList<QuadTreeForGroundNode> childrenToPack)
   {
      if (hasChildren)
      {
         childrenToPack.add(NW);
         childrenToPack.add(NE);
         childrenToPack.add(SE);
         childrenToPack.add(SW);
      }
   }

   public boolean hasChildren()
   {
      return hasChildren;
   }
   
   public boolean isEmpty()
   {
      return (!hasChildren && (leaf == null));
   }
   
   public QuadTreeForGroundPutResult put(QuadTreeForGroundPoint point)
   {
      if (this.hasChildren)
      {
         QuadTreeForGroundPutResult putResult = getChild(point.getX(), point.getY()).put(point);

         return putResult;
      }

      else if (this.leaf == null)
      {
         QuadTreeForGroundLeaf leaf = new QuadTreeForGroundLeaf(this, pointLimiter);
         leaf.addPoint(point);
         setLeaf(leaf);

         for (QuadTreeForGroundListener listener : listeners)
         {
            listener.nodeRemoved(id);
            listener.nodeAdded(id, bounds, (float) point.getX(), (float) point.getY(), (float) point.getZ());
         }

         QuadTreeForGroundPutResult quadTreePutResult = createNewResult();
         quadTreePutResult.addNode = true;
         quadTreePutResult.treeChanged = true;

         return quadTreePutResult;
      }


      else if (isAtSmallestResolution())
      {
         boolean updateChangedTree = updateLeafValueIfHeightIsAppropriate(point);

         QuadTreeForGroundPutResult quadTreePutResult = createNewResult();
         quadTreePutResult.treeChanged = updateChangedTree;

         if (updateChangedTree)
         {
            for (QuadTreeForGroundListener listener : listeners)
            {
               Point3D averagePoint = leaf.getAveragePoint();
               listener.nodeRemoved(id);
               listener.nodeAdded(id, bounds, (float) averagePoint.getX(), (float) averagePoint.getY(), (float) averagePoint.getZ());
            }

            quadTreePutResult.changedNode = true;
            quadTreePutResult.treeChanged = true;
         }

         else
         {
            quadTreePutResult.changedNode = false;
            quadTreePutResult.treeChanged = false;
         }

         return quadTreePutResult;
      }

      else if (isNotYetAtResonableResolution())
      {
         this.divide();

         return this.put(point);
      }

      Point3D averagePoint = leaf.getAveragePoint();
      if (Math.abs(averagePoint.getZ() - point.getZ()) < parameters.getHeightThreshold())
      {
         updateLeafValue(point);
         QuadTreeForGroundPutResult quadTreePutResult = createNewResult();

         return quadTreePutResult;
      }

      else
      {
         this.divide();

         return this.put(point);
      }
   }

   public String getID()
   {
      return id;
   }

   private boolean isAtSmallestResolution()
   {
      return Math.abs(bounds.maxX - bounds.minX) < parameters.getResolution();
   }

   private boolean isNotYetAtResonableResolution()
   {
      // TODO: Magic number 4.1. Do something about that...
      return Math.abs(bounds.maxX - bounds.minX) > 4.1 * parameters.getResolution();
   }

   public Box getBounds()
   {
      return this.bounds;
   }


   public void clear()
   {
      if (this.hasChildren)
      {
         this.NW.clear();
         this.NE.clear();
         this.SE.clear();
         this.SW.clear();
         this.NW = null;
         this.NE = null;
         this.SE = null;
         this.SW = null;
         this.hasChildren = false;
      }
      else
      {
         for (QuadTreeForGroundListener listener : listeners)
         {
            listener.nodeRemoved(this.getID());
         }
         
         if (this.leaf != null)
         {
            this.leaf.clear();
            setLeaf(null);
         }
      }
   }

   public void getClosestPoint(double xQuery, double yQuery, Point3D pointToPack)
   {
      PointAndDistance pointAndDistance = new PointAndDistance(pointToPack, Double.POSITIVE_INFINITY);
      getClosestPointAndDistance(xQuery, yQuery, pointAndDistance);
   }

   public void getClosestPointAndDistance(double x, double y, PointAndDistance bestSoFarToUpdate)
   {
      // exclude node if point is farther away than best distance in either axis
      if ((x < bounds.minX - bestSoFarToUpdate.getDistance()) || (x > bounds.maxX + bestSoFarToUpdate.getDistance())
              || (y < bounds.minY - bestSoFarToUpdate.getDistance()) || (y > bounds.maxY + bestSoFarToUpdate.getDistance()))
      {
         return;
      }

      if (this.hasChildren)
      {
         QuadTreeForGroundNode childAtXY = this.getChild(x, y);
         if (childAtXY != null)
            childAtXY.getClosestPointAndDistance(x, y, bestSoFarToUpdate);


         if (NE != childAtXY)
            NE.getClosestPointAndDistance(x, y, bestSoFarToUpdate);
         if (NW != childAtXY)
            NW.getClosestPointAndDistance(x, y, bestSoFarToUpdate);
         if (SE != childAtXY)
            SE.getClosestPointAndDistance(x, y, bestSoFarToUpdate);
         if (SW != childAtXY)
            SW.getClosestPointAndDistance(x, y, bestSoFarToUpdate);
      }

      else if (this.leaf != null)
      {
         if (isAtSmallestResolution())
         {
            leaf.getClosestPointAndDistanceUsingAverage(x, y, bestSoFarToUpdate);
         }
         else
         {
            leaf.getClosestPointAndDistanceUsingAverageHeight(x, y, bestSoFarToUpdate);
         }
      }
   }


   public void getAllPointsWithinDistance(double x, double y, double maxDistance, ArrayList<Point3D> pointsWithinDistanceToPack)
   {
      if (maxDistance < 0.0)
         return;

      if (this.hasChildren)
      {
         if (this.NW.bounds.calcDist(x, y) <= maxDistance)
         {
            this.NW.getAllPointsWithinDistance(x, y, maxDistance, pointsWithinDistanceToPack);
         }

         if (this.NE.bounds.calcDist(x, y) <= maxDistance)
         {
            this.NE.getAllPointsWithinDistance(x, y, maxDistance, pointsWithinDistanceToPack);
         }

         if (this.SE.bounds.calcDist(x, y) <= maxDistance)
         {
            this.SE.getAllPointsWithinDistance(x, y, maxDistance, pointsWithinDistanceToPack);
         }

         if (this.SW.bounds.calcDist(x, y) <= maxDistance)
         {
            this.SW.getAllPointsWithinDistance(x, y, maxDistance, pointsWithinDistanceToPack);
         }

         return;
      }

      if (this.leaf != null)
      {
         leaf.getAllPointsWithinDistance(x, y, maxDistance, pointsWithinDistanceToPack);
      }
   }

   public void getAllPointsWithBounds(Box bounds, ArrayList<Point3D> pointsWithinBoundsToPack)
   {
      if (this.hasChildren)
      {
         if (this.NW.bounds.intersects(bounds))
         {
            this.NW.getAllPointsWithBounds(bounds, pointsWithinBoundsToPack);
         }

         if (this.NE.bounds.intersects(bounds))
         {
            this.NE.getAllPointsWithBounds(bounds, pointsWithinBoundsToPack);
         }

         if (this.SE.bounds.intersects(bounds))
         {
            this.SE.getAllPointsWithBounds(bounds, pointsWithinBoundsToPack);
         }

         if (this.SW.bounds.intersects(bounds))
         {
            this.SW.getAllPointsWithBounds(bounds, pointsWithinBoundsToPack);
         }
      }

      else if (this.leaf != null)
      {
         if (bounds.containsOrEquals(this.bounds))
         {
            this.leaf.getAllPoints(pointsWithinBoundsToPack);
         }
         else
         {
            this.leaf.getAllPointsWithinBounds(bounds, pointsWithinBoundsToPack);
         }
      }
   }

   private void divide()
   {
      for (QuadTreeForGroundListener listener : listeners)
      {
         listener.nodeRemoved(id);
      }

      this.NW = new QuadTreeForGroundNode(id + "NW", (double) this.bounds.minX, (double) this.bounds.centreY, (double) this.bounds.centreX,
                                       (double) this.bounds.maxY, parameters, pointLimiter, this, defaultHeightWhenNoPonts, listeners);
      this.NE = new QuadTreeForGroundNode(id + "NE", (double) this.bounds.centreX, (double) this.bounds.centreY, (double) this.bounds.maxX,
                                       (double) this.bounds.maxY, parameters, pointLimiter, this, defaultHeightWhenNoPonts, listeners);
      this.SE = new QuadTreeForGroundNode(id + "SE", (double) this.bounds.centreX, (double) this.bounds.minY, (double) this.bounds.maxX,
                                       (double) this.bounds.centreY, parameters, pointLimiter, this, defaultHeightWhenNoPonts, listeners);
      this.SW = new QuadTreeForGroundNode(id + "SW", (double) this.bounds.minX, (double) this.bounds.minY, (double) this.bounds.centreX,
                                       (double) this.bounds.centreY, parameters, pointLimiter, this, defaultHeightWhenNoPonts, listeners);
      this.hasChildren = true;

      if (this.leaf != null)
      {
         ArrayList<QuadTreeForGroundPoint> points = this.leaf.getPoints();

         if (points != null)
         {
            for (QuadTreeForGroundPoint point3d : points)
            {
               this.put(point3d);
            }

         }
         setLeaf(null);
      }
   }


   private QuadTreeForGroundNode getChild(double x, double y)
   {
      if (this.hasChildren)
      {
         if (x < this.bounds.centreX)
         {
            if (y < this.bounds.centreY)
            {
               return this.SW;
            }

            return this.NW;
         }

         if (y < this.bounds.centreY)
         {
            return this.SE;
         }

         return this.NE;
      }

      return null;
   }

   public int getNumberOfChildren()
   {
      int count = 0;
      if (hasChildren)
      {
         if (NE != null)
         {
            count += NE.getNumberOfChildren();
         }

         if (NW != null)
         {
            count += NW.getNumberOfChildren();
         }

         if (SE != null)
         {
            count += SE.getNumberOfChildren();
         }

         if (SW != null)
         {
            count += SW.getNumberOfChildren();
         }
      }
      else
      {
         count = 1;
      }

      return count;
   }


   protected QuadTreeForGroundPutResult createNewResult()
   {
      return new QuadTreeForGroundPutResult();
   }

   protected void setLeaf(QuadTreeForGroundLeaf leaf)
   {
      this.leaf = leaf;
   }

   public QuadTreeForGroundLeaf getLeaf()
   {
      return leaf;
   }


   private double getAverageHeightOfLeaf()
   {
      Point3D averagePoint = leaf.getAveragePoint();

      return averagePoint.getZ();
   }

   private boolean updateLeafValueIfHeightIsAppropriate(QuadTreeForGroundPoint point)
   {
      double heightDifference = point.getZ() - getAverageHeightOfLeaf();

      if ((heightDifference > 0.0) && (heightDifference < parameters.getMaxMultiLevelZChangeToFilterNoise()))    // higher, but not more than maxMultiLevelZChangeToFilterNoise, then keep it.
      {
         updateLeafValue(point);

         return true;
      }
      else if ((heightDifference < 0.0) && (Math.abs(heightDifference) > parameters.getMaxMultiLevelZChangeToFilterNoise()))    // lower, but at least maxMultiLevelZChangeToFilterNoise, then keep it.
      {
         updateLeafValue(point);

         return true;
      }

      return false;
   }


   private void updateLeafValue(QuadTreeForGroundPoint point)
   {
      if (leaf.getNumberOfPoints() < parameters.getMaxSameHeightPointsPerNode())
      {
         leaf.addPoint(point);
      }

      else
      {
         //leaf.replaceARandomPoint(x, y, z);
         leaf.replaceLeastRecentPoint(point);
      }
   }

   public void checkRepInvarients()
   {
      boolean isLeaf = (leaf != null);
      if (isLeaf)
      {
         if (hasChildren)
            throw new RuntimeException();
         if (NE != null)
            throw new RuntimeException();
         if (NW != null)
            throw new RuntimeException();
         if (SE != null)
            throw new RuntimeException();
         if (SW != null)
            throw new RuntimeException();

         leaf.checkRepInvarients(this.bounds);
      }

      if (!hasChildren)
      {
         if (NE != null)
            throw new RuntimeException();
         if (NW != null)
            throw new RuntimeException();
         if (SE != null)
            throw new RuntimeException();
         if (SW != null)
            throw new RuntimeException();
      }
      else
      {
         this.bounds.containsOrEquals(NE.bounds);
         this.bounds.containsOrEquals(NW.bounds);
         this.bounds.containsOrEquals(SE.bounds);
         this.bounds.containsOrEquals(SW.bounds);

         NE.checkRepInvarients();
         NW.checkRepInvarients();
         SE.checkRepInvarients();
         SW.checkRepInvarients();
      }
   }

   public String toString()
   {
      return "QuadNode{" + "bounds=" + bounds + ", leaf=" + leaf + ", hasChildren=" + hasChildren + '}';
   }

   public void merge()
   {
      if(hasChildren)
      {
         if(NE.isEmpty() && NW.isEmpty() && SE.isEmpty() && SW.isEmpty())
         {
            NE = null;
            NW = null;
            SE = null;
            SW = null;
            hasChildren = false;
            parent.merge();
         }
      }
      else
      {
         leaf = null;
         parent.merge();
      }
      
   }

   public QuadTreeForGroundPutResult put(double x, double y, double z)
   {
      QuadTreeForGroundPoint point = new QuadTreeForGroundPoint(x, y, z);
      return put(point);
   }

}
