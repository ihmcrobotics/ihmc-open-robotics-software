package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class DiscreteFootstep
{
   private final LatticePoint latticePoint;
   private final RobotSide robotSide;

   private Point2D midFootPoint;
   private final int hashCode;

   private final List<DiscreteFootstep> childNodes = new ArrayList<>();

   public DiscreteFootstep(double x, double y)
   {
      this(x, y, 0.0, RobotSide.LEFT);
   }

   public DiscreteFootstep(double x, double y, double yaw, RobotSide robotSide)
   {
      this(new LatticePoint(x, y, yaw), robotSide);
   }

   public DiscreteFootstep(int xIndex, int yIndex, int yawIndex, RobotSide robotSide)
   {
      this(new LatticePoint(xIndex, yIndex, yawIndex), robotSide);
   }

   public DiscreteFootstep(LatticePoint latticePoint, RobotSide robotSide)
   {
      this.latticePoint = latticePoint;
      this.robotSide = robotSide;
      hashCode = computeHashCode(this);
   }

   public double getX()
   {
      return latticePoint.getX();
   }

   public double getY()
   {
      return latticePoint.getY();
   }

   public double getYaw()
   {
      return latticePoint.getYaw();
   }

   public int getXIndex()
   {
      return latticePoint.getXIndex();
   }

   public int getYIndex()
   {
      return latticePoint.getYIndex();
   }

   public int getYawIndex()
   {
      return latticePoint.getYawIndex();
   }

   public LatticePoint getLatticePoint()
   {
      return latticePoint;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public List<DiscreteFootstep> getChildNodes()
   {
      return childNodes;
   }

   public void addChildNode(DiscreteFootstep childNode)
   {
      if (!childNodes.contains(childNode))
      {
         childNodes.add(childNode);
      }
   }

   public double euclideanDistance(DiscreteFootstep other)
   {
      return Math.sqrt(euclideanDistanceSquared(other));
   }

   public double euclideanDistanceSquared(DiscreteFootstep other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      return dx * dx + dy * dy;
   }

   public int computeYawIndexDistance(DiscreteFootstep other)
   {
      int dYaw = Math.abs(latticePoint.getYawIndex() - other.getYawIndex());
      return Math.min(dYaw, LatticePoint.yawDivisions - dYaw);
   }

   public int computeXYManhattanDistance(DiscreteFootstep other)
   {
      int manhattanDistance = Math.abs(getXIndex() - other.getXIndex());
      manhattanDistance += Math.abs(getYIndex() - other.getYIndex());
      return manhattanDistance;
   }

   public int computeManhattanDistance(DiscreteFootstep other)
   {
      return computeXYManhattanDistance(other) + computeYawIndexDistance(other);
   }

   public boolean equalPosition(DiscreteFootstep other)
   {
      return getXIndex() == other.getXIndex() && getYIndex() == other.getYIndex();
   }

   public static DiscreteFootstep generateRandomFootstep(Random random, double minMaxXY)
   {
      return new DiscreteFootstep(EuclidCoreRandomTools.nextDouble(random, minMaxXY), EuclidCoreRandomTools.nextDouble(random, minMaxXY),
                                  EuclidCoreRandomTools.nextDouble(random, Math.PI), RobotSide.generateRandomRobotSide(random));
   }

   public static DiscreteFootstep generateRandomFootstep(Random random, double minMaxXY, RobotSide robotSide)
   {
      return new DiscreteFootstep(EuclidCoreRandomTools.nextDouble(random, minMaxXY), EuclidCoreRandomTools.nextDouble(random, minMaxXY),
                                  EuclidCoreRandomTools.nextDouble(random, Math.PI), robotSide);
   }

   public Point2D getOrComputeMidFootPoint(double stepWidth)
   {
      if (midFootPoint == null)
      {
         midFootPoint = computeMidFootPoint(this, stepWidth);
      }
      return midFootPoint;
   }

   private static Point2D computeMidFootPoint(DiscreteFootstep node, double idealStepWidth)
   {
      double w = idealStepWidth / 2.0;
      double vx = node.getRobotSide().negateIfRightSide(Math.sin(node.getYaw()) * w);
      double vy = -node.getRobotSide().negateIfRightSide(Math.cos(node.getYaw()) * w);
      return new Point2D(node.getX() + vx, node.getY() + vy);
   }

   @Override
   public int hashCode()
   {
      return hashCode;
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
      DiscreteFootstep other = (DiscreteFootstep) obj;
      if (robotSide != other.robotSide)
         return false;
      if(!latticePoint.equals(other.latticePoint))
         return false;
      return true;
   }

   private static int computeHashCode(DiscreteFootstep footstep)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((footstep.robotSide == null) ? 0 : footstep.robotSide.hashCode());
      result = prime * result + footstep.getLatticePoint().getXIndex();
      result = prime * result + footstep.getLatticePoint().getYIndex();
      result = prime * result + footstep.getLatticePoint().getYawIndex();
      return result;
   }

   @Override
   public String toString()
   {
      return "Node: x=" + getX() + ", y=" + getY() + ", yaw=" + getYaw() + ", side=" + robotSide.getLowerCaseName();
   }

}
