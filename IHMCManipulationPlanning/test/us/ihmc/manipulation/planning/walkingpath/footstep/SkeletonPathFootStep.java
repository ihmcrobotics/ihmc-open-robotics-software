package us.ihmc.manipulation.planning.walkingpath.footstep;

import java.awt.geom.Point2D;

import us.ihmc.robotics.robotSide.RobotSide;

public class SkeletonPathFootStep
{
   private RobotSide robotSide;
   private Point2D location;
   private double yawAngle;
   private int indexOfSegment;
   
   public SkeletonPathFootStep()
   {
      this.robotSide = RobotSide.LEFT;
      this.location = new Point2D.Double();
      this.yawAngle = 0.0;
   }
   
   public SkeletonPathFootStep(RobotSide robotSide, Point2D location, double yawAngle)
   {
      this.robotSide = robotSide;
      this.location = location;
      this.yawAngle = yawAngle;
   }
   
   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }
   
   public void setLocation(Point2D location)
   {
      this.location = location;
   }
   
   public void setYawAngle(double yawAngle)
   {
      this.yawAngle = yawAngle;
   }
   
   public void setIndexOfSegment(int index)
   {
      this.indexOfSegment = index;
   }
   
   public RobotSide getRobotSide()
   {
      return robotSide;
   }
   
   public Point2D getLocation()
   {
      return location;
   }
   
   public double getYawAngle()
   {
      return yawAngle;
   }
   
   public int getIndexOfSegment()
   {
      return this.indexOfSegment;
   }
}
