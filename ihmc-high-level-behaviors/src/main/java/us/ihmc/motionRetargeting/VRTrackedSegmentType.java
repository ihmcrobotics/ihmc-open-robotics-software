package us.ihmc.motionRetargeting;

import us.ihmc.robotics.robotSide.RobotSide;

public enum VRTrackedSegmentType
{
   // Hands defaults are 20 and 1. Reduce the orientation to 0.25 for the nub forearms
   LEFT_HAND("Left Hand", RobotSide.LEFT, 20.0, 0.25),
   RIGHT_HAND("Right Hand", RobotSide.RIGHT, 20.0, 0.25),
   LEFT_WRIST("Left Wrist", RobotSide.LEFT, 0.0, 1),
   RIGHT_WRIST("RightWrist", RobotSide.RIGHT, 0.0, 1),
   CHEST("Chest", null, 0.0, 10),
   WAIST("Waist", null, 0.0, 10);

   private String segmentName;
   private RobotSide robotSide;
   private double positionWeight;
   private double orientationWeight;

   VRTrackedSegmentType(String segmentName, RobotSide robotSide, double positionWeight, double orientationWeight)
   {
      this.segmentName = segmentName;
      this.robotSide = robotSide;
      this.positionWeight = positionWeight;
      this.orientationWeight = orientationWeight;
   }

   public String getSegmentName()
   {
      return segmentName;
   }

   public RobotSide getSegmentSide()
   {
      return robotSide;
   }

   public double getPositionWeight()
   {
      return positionWeight;
   }

   public double getOrientationWeight()
   {
      return orientationWeight;
   }
}