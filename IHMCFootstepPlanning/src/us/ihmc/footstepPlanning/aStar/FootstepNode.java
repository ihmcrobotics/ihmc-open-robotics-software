package us.ihmc.footstepPlanning.aStar;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepNode
{
   public static final double gridSizeX = 0.05;
   public static final double gridSizeY = 0.05;
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
      xIndex = (int) Math.round(x / gridSizeX);
      yIndex = (int) Math.round(y / gridSizeY);
      yawIndex = (int) Math.round(AngleTools.trimAngleMinusPiToPi(yaw) / gridSizeYaw);
      this.robotSide = robotSide;
   }

   public double getX()
   {
      return gridSizeX * xIndex;
   }

   public double getY()
   {
      return gridSizeY * yIndex;
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
      return "Node: x=" + getX() + ", y=" + getY();
   }

}
