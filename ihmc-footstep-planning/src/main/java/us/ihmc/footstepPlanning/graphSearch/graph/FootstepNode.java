package us.ihmc.footstepPlanning.graphSearch.graph;

import java.util.Random;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
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

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((robotSide == null) ? 0 : robotSide.hashCode());
      result = prime * result + xIndex;
      result = prime * result + yIndex;
      result = prime * result + yawIndex;
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
