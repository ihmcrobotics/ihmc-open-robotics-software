package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

public class FootstepNode
{
   public static final double gridSizeXY = 0.05;

   public static final double PRECISION = 0.05;
   public static final double INV_PRECISION = 1.0 / PRECISION;

   private final QuadrantDependentList<Integer> xIndices = new QuadrantDependentList<>();
   private final QuadrantDependentList<Integer> yIndices = new QuadrantDependentList<>();

   private Point2D midStancePoint;

   private final int hashCode;
   private final int planarRegionsHashCode;

   public FootstepNode(QuadrantDependentList<Point2DReadOnly> locations)
   {
      this(locations.get(RobotQuadrant.FRONT_LEFT), locations.get(RobotQuadrant.FRONT_RIGHT), locations.get(RobotQuadrant.HIND_LEFT),
           locations.get(RobotQuadrant.HIND_RIGHT));
   }

   public FootstepNode(Tuple2DReadOnly frontLeft, Tuple2DReadOnly frontRight, Tuple2DReadOnly hindLeft, Tuple2DReadOnly hindRight)
   {
      int xFrontLeftIndex = (int) Math.round(frontLeft.getX() / gridSizeXY);
      int yFrontLeftIndex = (int) Math.round(frontLeft.getY() / gridSizeXY);

      int xFrontRightIndex = (int) Math.round(frontRight.getX() / gridSizeXY);
      int yFrontRightIndex = (int) Math.round(frontRight.getY() / gridSizeXY);

      int xHindLeftIndex = (int) Math.round(hindLeft.getX() / gridSizeXY);
      int yHindLeftIndex = (int) Math.round(hindLeft.getY() / gridSizeXY);

      int xHindRightIndex = (int) Math.round(hindRight.getX() / gridSizeXY);
      int yHindRightIndex = (int) Math.round(hindRight.getY() / gridSizeXY);

      xIndices.put(RobotQuadrant.FRONT_LEFT, xFrontLeftIndex);
      yIndices.put(RobotQuadrant.FRONT_LEFT, yFrontLeftIndex);

      xIndices.put(RobotQuadrant.FRONT_RIGHT, xFrontRightIndex);
      yIndices.put(RobotQuadrant.FRONT_RIGHT, yFrontRightIndex);

      xIndices.put(RobotQuadrant.HIND_LEFT, xHindLeftIndex);
      yIndices.put(RobotQuadrant.HIND_LEFT, yHindLeftIndex);

      xIndices.put(RobotQuadrant.HIND_RIGHT, xHindRightIndex);
      yIndices.put(RobotQuadrant.HIND_RIGHT, yHindRightIndex);

      hashCode = computeHashCode(this);
      planarRegionsHashCode = computePlanarRegionsHashCode(this);
   }

   public double getX(RobotQuadrant robotQuadrant)
   {
      return gridSizeXY * xIndices.get(robotQuadrant);
   }

   public double getY(RobotQuadrant robotQuadrant)
   {
      return gridSizeXY * yIndices.get(robotQuadrant);
   }

   public int getXIndex(RobotQuadrant robotQuadrant)
   {
      return xIndices.get(robotQuadrant);
   }

   public int getYIndex(RobotQuadrant robotQuadrant)
   {
      return yIndices.get(robotQuadrant);
   }

   public double euclideanDistance(FootstepNode other)
   {
      return getOrComputeMidStancePoint().distance(other.getOrComputeMidStancePoint());
   }

   public double quadrantEuclideanDistance(RobotQuadrant robotQuadrant, FootstepNode other)
   {
      double dx = getX(robotQuadrant) - other.getX(robotQuadrant);
      double dy = getY(robotQuadrant) - other.getY(robotQuadrant);
      return Math.sqrt(MathTools.square(dx) + MathTools.square(dy));
   }

   public static FootstepNode generateRandomFootstepNode(Random random, double minMaxXY)
   {
      return new FootstepNode(
            new QuadrantDependentList<>(EuclidCoreRandomTools.nextPoint2D(random, minMaxXY), EuclidCoreRandomTools.nextPoint2D(random, minMaxXY),
                                        EuclidCoreRandomTools.nextPoint2D(random, minMaxXY), EuclidCoreRandomTools.nextPoint2D(random, minMaxXY)));
   }

   public Point2DReadOnly getOrComputeMidStancePoint()
   {
      if (midStancePoint == null)
      {
         midStancePoint = computeMidStancePoint(this);
      }
      return midStancePoint;
   }

   public static Point2D computeMidStancePoint(FootstepNode node)
   {
      double x = 0.0;
      double y = 0.0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         x += node.getX(robotQuadrant);
         y += node.getY(robotQuadrant);
      }

      x /= 4.0;
      y /= 4.0;
      return new Point2D(x, y);
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   public int getPlanarRegionsHashCode()
   {
      return planarRegionsHashCode;
   }

   private static int computeHashCode(FootstepNode node)
   {
      final int prime = 31;
      int result = 1;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         result = prime * result + node.getXIndex(robotQuadrant);
         result = prime * result + node.getYIndex(robotQuadrant);
      }
      return result;
   }

   public double getRoundedX(RobotQuadrant robotQuadrant)
   {
      return round(getX(robotQuadrant));
   }

   public double getRoundedY(RobotQuadrant robotQuadrant)
   {
      return round(getY(robotQuadrant));
   }

   public static int computePlanarRegionsHashCode(FootstepNode node)
   {
      final long prime = 31L;
      long bits = 1L;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         bits = prime * bits + Double.doubleToLongBits(node.getRoundedX(robotQuadrant));
         bits = prime * bits + Double.doubleToLongBits(node.getRoundedY(robotQuadrant));
      }
      return (int) (bits ^ bits >> 32);
   }

   public static double round(double value)
   {
      return Math.round(value * INV_PRECISION) * PRECISION;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      FootstepNode other = (FootstepNode) obj;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (xIndices.get(robotQuadrant) != other.xIndices.get(robotQuadrant))
            return false;
         if (yIndices.get(robotQuadrant) != other.yIndices.get(robotQuadrant))
            return false;
      }
      return true;
   }

   @Override
   public String toString()
   {
      String string = "Node: ";
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         string += "\n\t quadrant= " + robotQuadrant.getCamelCaseName() + ", x= " + getX(robotQuadrant) + ", y= " + getY(robotQuadrant);
      }
      return string;
   }
}
