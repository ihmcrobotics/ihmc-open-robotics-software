package us.ihmc.robotics.quadTree;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.euclid.tuple3D.Point3D;

/**
 * A quad tree for representing ground height maps.
 *
 * Uses a pretty straightforward implementation.
 * Does some filtering based on the parameters.
 * When a node gets to or smaller than the resolution, then the value of the node will be the average value of some of the points stored in the node.
 * As more than a certain amount of points get put into a small node, the points will start overwriting previous points randomly.
 *
 *
 * TODO: Later add functionality for clearing just a portion of the quad tree or for remembering the age of points and removing old points.
 * TODO: But for now, can only add to the quad tree, not take away.
 */
public class QuadTreeForGround
{
   private final ArrayList<QuadTreeForGroundListener> listeners = new ArrayList<QuadTreeForGroundListener>();
   private final QuadTreeForGroundNode root;
   private final Box bounds;

   private final QuadTreeForGroundParameters quadTreeParameters;
   private final Point3D nearestPointForHeightAt = new Point3D();
   private final QuadTreeForGroundPointLimiter pointLimiter;
   
   
   public QuadTreeForGround(double minX, double minY, double maxX, double maxY, double resolution, double heightThreshold,
                             double maxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode,
                             double maxAllowableXYDistanceForAPointToBeConsideredClose)
   {
      this(new Box(minX, minY, maxX, maxY), new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, -1));
   }

   public QuadTreeForGround(Box bounds, QuadTreeForGroundParameters quadTreeParameters)
   {
      this.bounds = bounds;
      this.quadTreeParameters = quadTreeParameters;
      
      if(quadTreeParameters.getMaximumNumberOfPoints() > 0)
      {
         pointLimiter = new QuadTreeForGroundPointLimiter(quadTreeParameters.getMaximumNumberOfPoints());
      }
      else 
      {
         pointLimiter = null;
      }
      this.root = new QuadTreeForGroundNode("root", bounds, quadTreeParameters, pointLimiter, null, Double.NaN, listeners);
   }

   public int getNumberOfPoints()
   {
      return pointLimiter.size();
   }
   
   public QuadTreeForGroundParameters getQuadTreeParameters()
   {
      return quadTreeParameters;
   }

   public void setHeightThreshold(double heightThreshold)
   {
      this.quadTreeParameters.setHeightThreshold(heightThreshold);
   }

   public void addQuadTreeListener(QuadTreeForGroundListener quadTreeListener)
   {
      listeners.add(quadTreeListener);
      ArrayList<QuadTreeForGroundNode> listOfCurrentNodes = getAllVisibleNodes();

      for (QuadTreeForGroundNode node : listOfCurrentNodes)
      {
         double x = node.getBounds().centreX;
         double y = node.getBounds().centreY;
         double z = 0.0f;

         QuadTreeForGroundLeaf leaf = node.getLeaf();
         if (leaf != null)
         {
            Point3D averagePoint = leaf.getAveragePoint();
            x = averagePoint.getX();
            y = averagePoint.getY();
            z = averagePoint.getZ();
         }

         quadTreeListener.nodeAdded(node.getID(), node.getBounds(), (float) x, (float) y, (float) z);
      }
   }

   private ArrayList<QuadTreeForGroundNode> getAllVisibleNodes()
   {
      ArrayList<QuadTreeForGroundNode> list = new ArrayList<QuadTreeForGroundNode>();
      root.getAllDescendantNodes(list);

      return list;
   }

   public synchronized QuadTreeForGroundPutResult put(double x, double y, double z)
   {
      for (QuadTreeForGroundListener listener : listeners)
      {
         listener.RawPointAdded((float) x, (float) y, (float) z);
      }

      return root.put(x, y, z);
   }
   
   public boolean isEmpty()
   {
      return root.isEmpty();
   }

   public synchronized void clear()
   {
      root.clear();
   }


   public synchronized double getHeightAtPoint(double x, double y)
   {
      if (!bounds.containsOrEquals(x, y))
         return Double.NaN;

      nearestPointForHeightAt.set(Double.NaN, Double.NaN, Double.NaN);
      PointAndDistance pointAndDistance = new PointAndDistance(nearestPointForHeightAt,
                                             quadTreeParameters.getMaxAllowableXYDistanceForAPointToBeConsideredClose());
      root.getClosestPointAndDistance(x, y, pointAndDistance);

      double heightToReturn = nearestPointForHeightAt.getZ();
      
      if (Double.isNaN(heightToReturn))
      {
         heightToReturn = root.getDefaultHeightWhenNoPoints();
      }
      return heightToReturn;
      
   }

   public synchronized void getClosestPoint(double xQuery, double yQuery, Point3D pointToPack)
   {
      root.getClosestPoint(xQuery, yQuery, pointToPack);
   }

   public synchronized void getAllPointsWithinDistance(double x, double y, double distance, ArrayList<Point3D> pointsWithinDistanceToPack)
   {
      this.root.getAllPointsWithinDistance(x, y, distance, pointsWithinDistanceToPack);
   }

   public synchronized void getAllPointsWithinBounds(Box bounds, ArrayList<Point3D> pointsWithinBoundsToPack)
   {
      this.root.getAllPointsWithBounds(bounds, pointsWithinBoundsToPack);
   }

   public double getMinX()
   {
      return root.getBounds().minX;
   }

   public double getMaxX()
   {
      return root.getBounds().maxX;
   }

   public double getMinY()
   {
      return root.getBounds().minY;
   }

   public double getMaxY()
   {
      return root.getBounds().maxY;
   }

   public synchronized int getNumberOfQuads()
   {
      return root.getNumberOfChildren();
   }

   public QuadTreeForGroundNode getRootNode()
   {
      return root;
   }
   
   public void setDefaultHeightWhenNoPoints(double defaultHeightWhenNoPonts)
   {
      root.setDefaultHeightWhenNoPoints(defaultHeightWhenNoPonts);
   }
   
   protected double getDefaultHeightWhenNoPoints()
   {
      return root.getDefaultHeightWhenNoPoints();
   }

   public synchronized void getStoredPoints(Collection<Point3D> points)
   {
      root.getAllSubTreePoints(points);
   }

   public synchronized void getCellAverageStoredPoints(Collection<Point3D> points)
   {
      root.getCellAverageSubTreePoints(points);
   }

   public void checkRepInvarients()
   {
      root.checkRepInvarients();
   }


}
