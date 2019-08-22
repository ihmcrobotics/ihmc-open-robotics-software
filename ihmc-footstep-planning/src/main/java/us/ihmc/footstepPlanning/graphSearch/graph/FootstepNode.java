package us.ihmc.footstepPlanning.graphSearch.graph;

import java.util.Objects;
import java.util.Random;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepNode
{
   private final LatticeNode latticeNode;
   private final RobotSide robotSide;

   private Point2D midFootPoint;
   private final int hashCode;

   public FootstepNode(double x, double y)
   {
      this(x, y, 0.0, RobotSide.LEFT);
   }

   public FootstepNode(double x, double y, double yaw, RobotSide robotSide)
   {
      this(new LatticeNode(x, y, yaw), robotSide);
   }

   public FootstepNode(LatticeNode latticeNode, RobotSide robotSide)
   {
      this.latticeNode = latticeNode;
      this.robotSide = robotSide;
      hashCode = computeHashCode(this);
   }

   public double getX()
   {
      return latticeNode.getX();
   }

   public double getY()
   {
      return latticeNode.getY();
   }

   public double getYaw()
   {
      return latticeNode.getYaw();
   }

   public int getXIndex()
   {
      return latticeNode.getXIndex();
   }

   public int getYIndex()
   {
      return latticeNode.getYIndex();
   }

   public int getYawIndex()
   {
      return latticeNode.getYawIndex();
   }

   public LatticeNode getLatticeNode()
   {
      return latticeNode;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double euclideanDistance(FootstepNode other)
   {
      return Math.sqrt(euclideanDistanceSquared(other));
   }

   public double euclideanDistanceSquared(FootstepNode other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      return dx * dx + dy * dy;
   }

   public static FootstepNode generateRandomFootstepNode(Random random, double minMaxXY)
   {
      return new FootstepNode(EuclidCoreRandomTools.nextDouble(random, minMaxXY), EuclidCoreRandomTools.nextDouble(random, minMaxXY),
                              EuclidCoreRandomTools.nextDouble(random, Math.PI), RobotSide.generateRandomRobotSide(random));
   }

   public Point2D getOrComputeMidFootPoint(double stepWidth)
   {
      if (midFootPoint == null)
      {
         midFootPoint = computeMidFootPoint(this, stepWidth);
      }
      return midFootPoint;
   }

   private static Point2D computeMidFootPoint(FootstepNode node, double idealStepWidth)
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
      FootstepNode other = (FootstepNode) obj;
      if (robotSide != other.robotSide)
         return false;
      if(!latticeNode.equals(other.latticeNode))
         return false;
      return true;
   }

   private static int computeHashCode(FootstepNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((node.robotSide == null) ? 0 : node.robotSide.hashCode());
      result = prime * result + node.getLatticeNode().getXIndex();
      result = prime * result + node.getLatticeNode().getYIndex();
      result = prime * result + node.getLatticeNode().getYawIndex();
      return result;
   }

   @Override
   public String toString()
   {
      return "Node: x=" + getX() + ", y=" + getY() + ", yaw=" + getYaw() + ", side=" + robotSide.getLowerCaseName();
   }

}
