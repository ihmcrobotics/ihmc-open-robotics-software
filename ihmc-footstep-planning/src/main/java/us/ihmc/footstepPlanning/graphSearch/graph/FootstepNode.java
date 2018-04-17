package us.ihmc.footstepPlanning.graphSearch.graph;

import java.util.Random;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepNode
{
   public static final double gridSizeXY = 0.05;
   public static final double gridSizeYaw = Math.PI / 18.0;

   private final int xIndex;
   private final int yIndex;
   private final int yawIndex;
   private final RobotSide robotSide;

   private Point2D midFootPoint;

   private final int hashCode;

   public FootstepNode(double x, double y)
   {
      this(x, y, 0.0, RobotSide.LEFT);
   }

   public FootstepNode(double x, double y, double yaw, RobotSide robotSide)
   {
      xIndex = (int) Math.round(x / gridSizeXY);
      yIndex = (int) Math.round(y / gridSizeXY);
      yawIndex = (int) Math.round(AngleTools.trimAngleMinusPiToPi(yaw) / gridSizeYaw);
      this.robotSide = robotSide;

      hashCode = computeHashCode(this);
   }

   public double getX()
   {
      return gridSizeXY * xIndex;
   }

   public double getY()
   {
      return gridSizeXY * yIndex;
   }

   public double getYaw()
   {
      return gridSizeYaw * yawIndex;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double euclideanDistance(FootstepNode other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      return Math.sqrt(dx * dx + dy * dy);
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

   public static Point2D computeMidFootPoint(FootstepNode node, double idealStepWidth)
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

   private static int computeHashCode(FootstepNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((node.robotSide == null) ? 0 : node.robotSide.hashCode());
      result = prime * result + node.xIndex;
      result = prime * result + node.yIndex;
      result = prime * result + node.yawIndex;
      return result;
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
      if (xIndex != other.xIndex)
         return false;
      if (yIndex != other.yIndex)
         return false;
      if (yawIndex != other.yawIndex)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "Node: x=" + getX() + ", y=" + getY() + ", yaw=" + getYaw() + ", side=" + robotSide.getLowerCaseName();
   }

}
