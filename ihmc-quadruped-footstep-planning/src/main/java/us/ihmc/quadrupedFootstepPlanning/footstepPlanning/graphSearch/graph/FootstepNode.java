package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepNode
{
   public static double gridSizeXY = 0.06;
   public static double gridSizeYaw = 0.1;

   private final QuadrantDependentList<Integer> xIndices = new QuadrantDependentList<>();
   private final QuadrantDependentList<Integer> yIndices = new QuadrantDependentList<>();
   private final int yawIndex;

   private final QuadrantDependentList<Double> xPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<Double> yPositions = new QuadrantDependentList<>();
   private final double stepYaw;

   private final double nominalStanceLength;
   private final double nominalStanceWidth;

   private Point2D xGaitCenterPoint;

   private final int hashCode;

   private final RobotQuadrant movingQuadrant;

   public FootstepNode(FootstepNode other)
   {
      this(other.getMovingQuadrant(), other.getX(RobotQuadrant.FRONT_LEFT), other.getY(RobotQuadrant.FRONT_LEFT), other.getX(RobotQuadrant.FRONT_RIGHT),
           other.getY(RobotQuadrant.FRONT_RIGHT), other.getX(RobotQuadrant.HIND_LEFT), other.getY(RobotQuadrant.HIND_LEFT), other.getX(RobotQuadrant.HIND_RIGHT),
           other.getY(RobotQuadrant.HIND_RIGHT), other.stepYaw, other.nominalStanceLength, other.nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, QuadrantDependentList<? extends Point2DReadOnly> locations, double stepYaw, double nominalStanceLength,
                       double nominalStanceWidth)
   {
      this(movingQuadrant, locations.get(RobotQuadrant.FRONT_LEFT), locations.get(RobotQuadrant.FRONT_RIGHT), locations.get(RobotQuadrant.HIND_LEFT),
           locations.get(RobotQuadrant.HIND_RIGHT), stepYaw, nominalStanceLength, nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, Tuple2DReadOnly frontLeft, Tuple2DReadOnly frontRight, Tuple2DReadOnly hindLeft, Tuple2DReadOnly hindRight,
                       double stepYaw, double nominalStanceLength, double nominalStanceWidth)
   {
      this(movingQuadrant, frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(), hindRight.getX(),
           hindRight.getY(), stepYaw, nominalStanceLength, nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX,
                       double hindLeftY, double hindRightX, double hindRightY,  double stepYaw, double nominalStanceLength, double nominalStanceWidth)
   {
      this(movingQuadrant, snapToGrid(frontLeftX), snapToGrid(frontLeftY), snapToGrid(frontRightX), snapToGrid(frontRightY), snapToGrid(hindLeftX),
           snapToGrid(hindLeftY), snapToGrid(hindRightX), snapToGrid(hindRightY), snapToYawGrid(stepYaw), nominalStanceLength, nominalStanceWidth);
   }

   public FootstepNode(RobotQuadrant movingQuadrant, int xFrontLeftIndex, int yFrontLeftIndex, int xFrontRightIndex, int yFrontRightIndex, int xHindLeftIndex,
                       int yHindLeftIndex, int xHindRightIndex, int yHindRightIndex, int yawIndex, double nominalStanceLength, double nominalStanceWidth)
   {
      this.movingQuadrant = movingQuadrant;
      this.nominalStanceLength = nominalStanceLength;
      this.nominalStanceWidth = nominalStanceWidth;

      xIndices.put(RobotQuadrant.FRONT_LEFT, xFrontLeftIndex);
      yIndices.put(RobotQuadrant.FRONT_LEFT, yFrontLeftIndex);

      xIndices.put(RobotQuadrant.FRONT_RIGHT, xFrontRightIndex);
      yIndices.put(RobotQuadrant.FRONT_RIGHT, yFrontRightIndex);

      xIndices.put(RobotQuadrant.HIND_LEFT, xHindLeftIndex);
      yIndices.put(RobotQuadrant.HIND_LEFT, yHindLeftIndex);

      xIndices.put(RobotQuadrant.HIND_RIGHT, xHindRightIndex);
      yIndices.put(RobotQuadrant.HIND_RIGHT, yHindRightIndex);

      this.yawIndex = yawIndex;

      double xFrontLeft = gridSizeXY * xFrontLeftIndex;
      double yFrontLeft = gridSizeXY * yFrontLeftIndex;

      double xFrontRight = gridSizeXY * xFrontRightIndex;
      double yFrontRight = gridSizeXY * yFrontRightIndex;

      double xHindLeft = gridSizeXY * xHindLeftIndex;
      double yHindLeft = gridSizeXY * yHindLeftIndex;

      double xHindRight = gridSizeXY * xHindRightIndex;
      double yHindRight = gridSizeXY * yHindRightIndex;

      xPositions.put(RobotQuadrant.FRONT_LEFT, xFrontLeft);
      yPositions.put(RobotQuadrant.FRONT_LEFT, yFrontLeft);

      xPositions.put(RobotQuadrant.FRONT_RIGHT, xFrontRight);
      yPositions.put(RobotQuadrant.FRONT_RIGHT, yFrontRight);

      xPositions.put(RobotQuadrant.HIND_LEFT, xHindLeft);
      yPositions.put(RobotQuadrant.HIND_LEFT, yHindLeft);

      xPositions.put(RobotQuadrant.HIND_RIGHT, xHindRight);
      yPositions.put(RobotQuadrant.HIND_RIGHT, yHindRight);

      stepYaw = gridSizeYaw * yawIndex;

      hashCode = computeHashCode(this);
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

   public double getStepYaw()
   {
      return stepYaw;
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

   public int getYawIndex()
   {
      return yawIndex;
   }

   public double euclideanDistance(FootstepNode other)
   {
      return getOrComputeXGaitCenterPoint().distance(other.getOrComputeXGaitCenterPoint());
   }

   public double quadrantEuclideanDistance(RobotQuadrant robotQuadrant, FootstepNode other)
   {
      double dx = getX(robotQuadrant) - other.getX(robotQuadrant);
      double dy = getY(robotQuadrant) - other.getY(robotQuadrant);
      return Math.sqrt(MathTools.square(dx) + MathTools.square(dy));
   }

   public Point2DReadOnly getOrComputeXGaitCenterPoint()
   {
      if (xGaitCenterPoint == null)
      {
         xGaitCenterPoint = computeXGaitCenterPoint(this);
      }
      return xGaitCenterPoint;
   }

   // TODO test me
   private static Point2D computeXGaitCenterPoint(FootstepNode node)
   {
      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      Vector2D offset = new Vector2D(movingQuadrant.getEnd().negateIfFrontEnd(node.getNominalStanceLength()),
                                     movingQuadrant.getSide().negateIfLeftSide(node.getNominalStanceWidth()));
      offset.scale(0.5);
      Orientation3DReadOnly rotation = new AxisAngle(node.getStepYaw(), 0.0, 0.0);
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

   private static int computeHashCode(FootstepNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((node.getMovingQuadrant() == null) ? 0 : node.getMovingQuadrant().hashCode());
      result = prime * result + node.getXIndex(node.getMovingQuadrant());
      result = prime * result + node.getYIndex(node.getMovingQuadrant());
      result = prime * result + node.getYawIndex();
      return result;
   }


   public static int snapToGrid(double location)
   {
      return (int) Math.round(location / gridSizeXY);
   }

   public static int snapToYawGrid(double yaw)
   {
      return (int) Math.round(yaw / gridSizeYaw);
   }

   public static Point2DReadOnly snapPointToGrid(Point2DReadOnly pointToSnap)
   {
      Point2D snappedPoint = new Point2D();
      snappedPoint.set(FootstepNode.gridSizeXY * snapToGrid(pointToSnap.getX()), FootstepNode.gridSizeXY * snapToGrid(pointToSnap.getY()));
      return snappedPoint;
   }

   public static void snapPointToGrid(Point2D pointToSnap)
   {
      pointToSnap.set(FootstepNode.gridSizeXY * snapToGrid(pointToSnap.getX()), FootstepNode.gridSizeXY * snapToGrid(pointToSnap.getY()));
   }

   public boolean quadrantGeometricallyEquals(FootstepNode other)
   {
      return quadrantGeometricallyEquals(movingQuadrant, other);
   }

   public boolean quadrantGeometricallyEquals(RobotQuadrant quadrant, FootstepNode other)
   {
      if (this == other)
         return true;
      if (other == null)
         return false;

      if (getXIndex(quadrant) != other.getXIndex(quadrant))
         return false;

      return getYIndex(quadrant) == other.getYIndex(quadrant);
   }

   public boolean xGaitGeometricallyEquals(FootstepNode other)
   {
      if (this == other)
         return true;
      if (other == null)
         return false;

      if (!getOrComputeXGaitCenterPoint().geometricallyEquals(other.getOrComputeXGaitCenterPoint(), gridSizeXY))
         return false;

      return getYawIndex() == other.getYawIndex();
   }

   public boolean geometricallyEquals(FootstepNode other)
   {
      if (this == other)
         return true;
      if (other == null)
         return false;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (getXIndex(robotQuadrant) != other.getXIndex(robotQuadrant))
            return false;
         if (getYIndex(robotQuadrant) != other.getYIndex(robotQuadrant))
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

      if (getXIndex(getMovingQuadrant()) != other.getXIndex(getMovingQuadrant()))
         return false;
      if (getYIndex(getMovingQuadrant()) != other.getYIndex(getMovingQuadrant()))
         return false;
      if (getYawIndex() != other.getYawIndex())
         return false;
      return getMovingQuadrant() == other.getMovingQuadrant();
   }

   @Override
   public String toString()
   {
      String string = "Node: ";
      string += "\n\t moving quadrant = " + getMovingQuadrant();
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

   public static FootstepNode constructNodeFromOtherNode(RobotQuadrant newMovingQuadrant, Point2DReadOnly newPosition, double newYaw, FootstepNode previousNode)
   {
      return constructNodeFromOtherNode(newMovingQuadrant, newPosition.getX(), newPosition.getY(), newYaw, previousNode);
   }

   public static FootstepNode constructNodeFromOtherNode(RobotQuadrant newMovingQuadrant, Point2DReadOnly newPosition, double newYaw, FootstepNode previousNode,
                                                         double nominalStanceLength, double nominalStanceWidth)
   {
      return constructNodeFromOtherNode(newMovingQuadrant, newPosition.getX(), newPosition.getY(), newYaw, previousNode, nominalStanceLength, nominalStanceWidth);
   }

   public static FootstepNode constructNodeFromOtherNode(RobotQuadrant newMovingQuadrant, double newXPosition, double newYPosition, double newYaw,
                                                         FootstepNode previousNode)
   {
      return constructNodeFromOtherNode(newMovingQuadrant, FootstepNode.snapToGrid(newXPosition), FootstepNode.snapToGrid(newYPosition),
                                        FootstepNode.snapToYawGrid(newYaw), previousNode, previousNode.getNominalStanceLength(),
                                        previousNode.getNominalStanceWidth());
   }

   public static FootstepNode constructNodeFromOtherNode(RobotQuadrant newMovingQuadrant, double newXPosition, double newYPosition, double newYaw,
                                                         FootstepNode previousNode, double nominalStanceLength, double nominalStanceWidth)
   {
      return constructNodeFromOtherNode(newMovingQuadrant, FootstepNode.snapToGrid(newXPosition), FootstepNode.snapToGrid(newYPosition),
                                        FootstepNode.snapToYawGrid(newYaw), previousNode, nominalStanceLength, nominalStanceWidth);
   }

   public static FootstepNode constructNodeFromOtherNode(RobotQuadrant newMovingQuadrant, int newXIndex, int newYIndex, int newYawIndex,
                                                         FootstepNode previousNode)
   {
      return constructNodeFromOtherNode(newMovingQuadrant, newXIndex, newYIndex, newYawIndex, previousNode, previousNode.getNominalStanceLength(),
                                        previousNode.getNominalStanceWidth());
   }

   public static FootstepNode constructNodeFromOtherNode(RobotQuadrant newMovingQuadrant, int newXIndex, int newYIndex, int newYawIndex, FootstepNode previousNode,
                                                         double nominalStanceLength, double nominalStanceWidth)
   {
      int xFrontLeft = previousNode.getXIndex(RobotQuadrant.FRONT_LEFT);
      int yFrontLeft = previousNode.getYIndex(RobotQuadrant.FRONT_LEFT);
      int xFrontRight = previousNode.getXIndex(RobotQuadrant.FRONT_RIGHT);
      int yFrontRight = previousNode.getYIndex(RobotQuadrant.FRONT_RIGHT);
      int xHindLeft = previousNode.getXIndex(RobotQuadrant.HIND_LEFT);
      int yHindLeft = previousNode.getYIndex(RobotQuadrant.HIND_LEFT);
      int xHindRight = previousNode.getXIndex(RobotQuadrant.HIND_RIGHT);
      int yHindRight = previousNode.getYIndex(RobotQuadrant.HIND_RIGHT);

      switch (newMovingQuadrant)
      {
      case FRONT_LEFT:
         xFrontLeft = newXIndex;
         yFrontLeft = newYIndex;
         break;
      case FRONT_RIGHT:
         xFrontRight = newXIndex;
         yFrontRight = newYIndex;
         break;
      case HIND_LEFT:
         xHindLeft = newXIndex;
         yHindLeft = newYIndex;
         break;
      case HIND_RIGHT:
         xHindRight = newXIndex;
         yHindRight = newYIndex;
         break;
      }
      return new FootstepNode(newMovingQuadrant, xFrontLeft, yFrontLeft, xFrontRight, yFrontRight, xHindLeft, yHindLeft, xHindRight, yHindRight,
                              newYawIndex, nominalStanceLength, nominalStanceWidth);
   }
}
