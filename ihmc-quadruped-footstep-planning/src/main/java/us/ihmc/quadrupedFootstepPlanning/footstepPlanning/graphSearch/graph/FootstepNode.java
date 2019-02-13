package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

public class FootstepNode
{
   public static final double gridSizeXY = 0.08;

   public static final double PRECISION = 0.05;
   public static final double INV_PRECISION = 1.0 / PRECISION;

   private final QuadrantDependentList<Integer> xIndices = new QuadrantDependentList<>();
   private final QuadrantDependentList<Integer> yIndices = new QuadrantDependentList<>();

   private final QuadrantDependentList<Double> xPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<Double> yPositions = new QuadrantDependentList<>();
   private final double nominalYaw;

   private final double nominalStanceLength;
   private final double nominalStanceWidth;

   private Point2D midStancePoint;
   private Point2D xGaitCenterPoint;

   private final int hashCode;
   private final int planarRegionsHashCode;

   private final RobotQuadrant movingQuadrant;

   public FootstepNode(FootstepNode other)
   {
      this(other.getMovingQuadrant(), other.getX(RobotQuadrant.FRONT_LEFT), other.getY(RobotQuadrant.FRONT_LEFT), other.getX(RobotQuadrant.FRONT_RIGHT),
           other.getY(RobotQuadrant.FRONT_RIGHT), other.getX(RobotQuadrant.HIND_LEFT), other.getY(RobotQuadrant.HIND_LEFT), other.getX(RobotQuadrant.HIND_RIGHT),
           other.getY(RobotQuadrant.HIND_RIGHT), other.nominalStanceLength, other.nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, QuadrantDependentList<Point2DReadOnly> locations, double nominalStanceLength,
                       double nominalStanceWidth)
   {
      this(movingQuadrant, locations.get(RobotQuadrant.FRONT_LEFT), locations.get(RobotQuadrant.FRONT_RIGHT), locations.get(RobotQuadrant.HIND_LEFT),
           locations.get(RobotQuadrant.HIND_RIGHT), nominalStanceLength, nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, Tuple2DReadOnly frontLeft, Tuple2DReadOnly frontRight, Tuple2DReadOnly hindLeft, Tuple2DReadOnly hindRight,
                       double nominalStanceLength, double nominalStanceWidth)
   {
      this(movingQuadrant, frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(), hindRight.getX(),
           hindRight.getY(), nominalStanceLength, nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX,
                       double hindLeftY, double hindRightX, double hindRightY, double nominalStanceLength, double nominalStanceWidth)
   {
      this.movingQuadrant = movingQuadrant;
      this.nominalStanceLength = nominalStanceLength;
      this.nominalStanceWidth = nominalStanceWidth;

      int xFrontLeftIndex = snapToGrid(frontLeftX);
      int yFrontLeftIndex = snapToGrid(frontLeftY);

      int xFrontRightIndex = snapToGrid(frontRightX);
      int yFrontRightIndex = snapToGrid(frontRightY);

      int xHindLeftIndex = snapToGrid(hindLeftX);
      int yHindLeftIndex = snapToGrid(hindLeftY);

      int xHindRightIndex = snapToGrid(hindRightX);
      int yHindRightIndex = snapToGrid(hindRightY);

      xIndices.put(RobotQuadrant.FRONT_LEFT, xFrontLeftIndex);
      yIndices.put(RobotQuadrant.FRONT_LEFT, yFrontLeftIndex);

      xIndices.put(RobotQuadrant.FRONT_RIGHT, xFrontRightIndex);
      yIndices.put(RobotQuadrant.FRONT_RIGHT, yFrontRightIndex);

      xIndices.put(RobotQuadrant.HIND_LEFT, xHindLeftIndex);
      yIndices.put(RobotQuadrant.HIND_LEFT, yHindLeftIndex);

      xIndices.put(RobotQuadrant.HIND_RIGHT, xHindRightIndex);
      yIndices.put(RobotQuadrant.HIND_RIGHT, yHindRightIndex);

      xPositions.put(RobotQuadrant.FRONT_LEFT,  gridSizeXY * xFrontLeftIndex);
      yPositions.put(RobotQuadrant.FRONT_LEFT,  gridSizeXY * yFrontLeftIndex);

      xPositions.put(RobotQuadrant.FRONT_RIGHT,  gridSizeXY * xFrontRightIndex);
      yPositions.put(RobotQuadrant.FRONT_RIGHT,  gridSizeXY * yFrontRightIndex);

      xPositions.put(RobotQuadrant.HIND_LEFT,  gridSizeXY * xHindLeftIndex);
      yPositions.put(RobotQuadrant.HIND_LEFT,  gridSizeXY * yHindLeftIndex);

      xPositions.put(RobotQuadrant.HIND_RIGHT,  gridSizeXY * xHindRightIndex);
      yPositions.put(RobotQuadrant.HIND_RIGHT,  gridSizeXY * yHindRightIndex);

      this.nominalYaw = computeNominalYaw(gridSizeXY * xFrontLeftIndex, gridSizeXY * yFrontLeftIndex, gridSizeXY * xFrontRightIndex,
                                          gridSizeXY * yFrontRightIndex, gridSizeXY * xHindLeftIndex, gridSizeXY * yHindLeftIndex,
                                          gridSizeXY * xHindRightIndex, gridSizeXY * yHindRightIndex);

      hashCode = computeHashCode(this);
      planarRegionsHashCode = computePlanarRegionsHashCode(this);
   }

   public RobotQuadrant getMovingQuadrant()
   {
      return movingQuadrant;
   }

   public double getX(RobotQuadrant robotQuadrant)
   {
      return xPositions.get(robotQuadrant);
   }

   public double getY(RobotQuadrant robotQuadrant)
   {
      return yPositions.get(robotQuadrant);
   }

   public double getNominalYaw()
   {
      return nominalYaw;
   }

   public double getNominalStanceLength()
   {
      return nominalStanceLength;
   }

   public double getNominalStanceWidth()
   {
      return nominalStanceWidth;
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
      return new FootstepNode(RobotQuadrant.generateRandomRobotQuadrant(random),
                              new QuadrantDependentList<>(EuclidCoreRandomTools.nextPoint2D(random, minMaxXY), EuclidCoreRandomTools.nextPoint2D(random, minMaxXY),
                                        EuclidCoreRandomTools.nextPoint2D(random, minMaxXY), EuclidCoreRandomTools.nextPoint2D(random, minMaxXY)), 1.0, 0.5);
   }

   public Point2DReadOnly getOrComputeMidStancePoint()
   {
      if (midStancePoint == null)
      {
         midStancePoint = computeMidStancePoint(this);
      }
      return midStancePoint;
   }

   public Point2DReadOnly getOrComputeXGaitCenterPoint()
   {
      if (xGaitCenterPoint == null)
      {
         xGaitCenterPoint = computeXGaitCenterPoint(this);
      }
      return xGaitCenterPoint;
   }

   private static Point2D computeMidStancePoint(FootstepNode node)
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

   private static Point2D computeXGaitCenterPoint(FootstepNode node)
   {
      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      Vector2D offset = new Vector2D(movingQuadrant.getEnd().negateIfFrontEnd(node.nominalStanceLength), movingQuadrant.getSide().negateIfLeftSide(node.nominalStanceWidth));
      offset.scale(0.5);
      Orientation3DReadOnly rotation = new AxisAngle(node.getNominalYaw(), 0.0, 0.0);
      rotation.transform(offset);

      Point2D newPoint =  new Point2D(node.getX(movingQuadrant), node.getY(movingQuadrant));
      newPoint.add(offset);

      return newPoint;
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
//      for (RobotQuadrant robotQuadrant)
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

   public static int snapToGrid(double location)
   {
      return (int) Math.round(location / gridSizeXY);
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

   public boolean completelyEquals(Object obj)
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
   public boolean equals(Object obj)
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
      if (yIndices.get(movingQuadrant) != other.yIndices.get(movingQuadrant))
         return false;
      return movingQuadrant == other.movingQuadrant;
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
