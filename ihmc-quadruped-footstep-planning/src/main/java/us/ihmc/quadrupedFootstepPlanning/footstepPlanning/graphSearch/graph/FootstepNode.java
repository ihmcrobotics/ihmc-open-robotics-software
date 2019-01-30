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

   private double nominalYaw;

   private Point2D midStancePoint;

   private final int hashCode;
   private final int planarRegionsHashCode;

   private final RobotQuadrant movingQuadrant;

   public FootstepNode(RobotQuadrant movingQuadrant, QuadrantDependentList<Point2DReadOnly> locations)
   {
      this(movingQuadrant, locations.get(RobotQuadrant.FRONT_LEFT), locations.get(RobotQuadrant.FRONT_RIGHT), locations.get(RobotQuadrant.HIND_LEFT),
           locations.get(RobotQuadrant.HIND_RIGHT));
   }

   public FootstepNode(RobotQuadrant movingQuadrant, Tuple2DReadOnly frontLeft, Tuple2DReadOnly frontRight, Tuple2DReadOnly hindLeft, Tuple2DReadOnly hindRight)
   {
      this(movingQuadrant, frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(), hindRight.getX(),
           hindRight.getY());
   }

   public FootstepNode(RobotQuadrant movingQuadrant, double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX,
                       double hindLeftY, double hindRightX, double hindRightY)
   {
      this.movingQuadrant = movingQuadrant;

      int xFrontLeftIndex = (int) Math.round(frontLeftX / gridSizeXY);
      int yFrontLeftIndex = (int) Math.round(frontLeftY / gridSizeXY);

      int xFrontRightIndex = (int) Math.round(frontRightX / gridSizeXY);
      int yFrontRightIndex = (int) Math.round(frontRightY / gridSizeXY);

      int xHindLeftIndex = (int) Math.round(hindLeftX / gridSizeXY);
      int yHindLeftIndex = (int) Math.round(hindLeftY / gridSizeXY);

      int xHindRightIndex = (int) Math.round(hindRightX / gridSizeXY);
      int yHindRightIndex = (int) Math.round(hindRightY / gridSizeXY);

      nominalYaw = computeNominalYaw(frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY);

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

   public RobotQuadrant getMovingQuadrant()
   {
      return movingQuadrant;
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

   public double getNominalYaw()
   {
      return nominalYaw;
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
      return new FootstepNode(RobotQuadrant.generateRandomRobotQuadrant(random),
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
      result = prime * result + ((node.movingQuadrant == null) ? 0 : node.movingQuadrant.hashCode());
      /*
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         result = prime * result + node.getXIndex(robotQuadrant);
         result = prime * result + node.getYIndex(robotQuadrant);
      }
      */
      result = prime * result + node.getXIndex(node.movingQuadrant);
      result = prime * result + node.getYIndex(node.movingQuadrant);
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
      Point2DReadOnly midstancePoint = node.getOrComputeMidStancePoint();
      return computePlanarRegionsHashCode(round(midstancePoint.getX()), round(midstancePoint.getY()));
   }

   public static int computePlanarRegionsHashCode(double roundedX, double roundedY)
   {
      final long prime = 31L;
      long bits = 1L;
      bits = prime * bits + Double.doubleToLongBits(roundedX);
      bits = prime * bits + Double.doubleToLongBits(roundedY);

      return (int) (bits ^ bits >> 32);
   }

   public static double round(double value)
   {
      return Math.round(value * INV_PRECISION) * PRECISION;
   }

   public boolean quadrantGeometricallyEquals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      FootstepNode other = (FootstepNode) obj;


      if (xIndices.get(movingQuadrant) != other.xIndices.get(movingQuadrant))
         return false;

      return yIndices.get(movingQuadrant) == other.yIndices.get(movingQuadrant);
   }

   public boolean midstanceGeometricallyEquals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      FootstepNode other = (FootstepNode) obj;

      return getOrComputeMidStancePoint().geometricallyEquals((other).getOrComputeMidStancePoint(), PRECISION);
   }

   public boolean geometricallyEquals(Object obj)
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
         if (movingQuadrant != other.movingQuadrant)
            return false;
      }
      return true;
   }

   @Override
   public String toString()
   {
      String string = "Node: ";
      string += "\n\t moving quadrant = " + movingQuadrant;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         string += "\n\t quadrant= " + robotQuadrant.getCamelCaseName() + ", x= " + getX(robotQuadrant) + ", y= " + getY(robotQuadrant);
      }
      return string;
   }


   public static double computeNominalYaw(double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX, double hindLeftY,
                                       double hindRightX, double hindRightY)
   {
      double deltaX = frontLeftX - hindLeftX;
      double deltaY = frontLeftY - hindLeftY;

      deltaX += frontRightX - hindRightX;
      deltaY += frontRightY - hindRightY;

      return Math.atan2(deltaY, deltaX);
   }
}
