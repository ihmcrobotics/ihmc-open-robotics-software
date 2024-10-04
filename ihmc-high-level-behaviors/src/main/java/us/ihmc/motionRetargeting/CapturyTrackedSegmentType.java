package us.ihmc.motionRetargeting;

import us.ihmc.robotics.robotSide.RobotSide;

public enum CapturyTrackedSegmentType
{
   LEFT_HAND("leftHand", RobotSide.LEFT, 0.0, 1.0), // Defaults are 20 and 1. Reduce the orientation for the knob arms
   RIGHT_HAND("rightHand", RobotSide.RIGHT, 0.0, 1.0),
   LEFT_UPPERARM("leftUpperArm", RobotSide.LEFT, 0.0, 1.0),
   RIGHT_UPPERARM("rightUpperArm", RobotSide.RIGHT, 0.0, 1.0),
   LEFT_FOREARM("leftForeArm", RobotSide.LEFT, 0.0, 1.0),
   RIGHT_FOREARM("rightForeArm", RobotSide.RIGHT, 0.0, 1.0),
   CHEST("chest", null, 0.0, 10);

   private String segmentName;
   private RobotSide robotSide;
   private double positionWeight;
   private double orientationWeight;

   CapturyTrackedSegmentType(String segmentName, RobotSide robotSide, double positionWeight, double orientationWeight)
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

   public static CapturyTrackedSegmentType toHand(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? LEFT_HAND : RIGHT_HAND;
   }

   public static CapturyTrackedSegmentType toForearm(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? LEFT_FOREARM : RIGHT_FOREARM;
   }

   public static CapturyTrackedSegmentType toUpperArm(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? LEFT_UPPERARM : RIGHT_UPPERARM;
   }

   public static CapturyTrackedSegmentType toChest()
   {
      return CHEST;
   }
}
