package us.ihmc.motionRetargeting;

import us.ihmc.robotics.robotSide.RobotSide;

public enum VRTrackedSegmentType
{
   LEFT_HAND("leftHand", RobotSide.LEFT, 20.0, 0.25), // Defaults are 20 and 1. Reduce the orientation for the knob arms
   RIGHT_HAND("rightHand", RobotSide.RIGHT, 20.0, 0.25),
   LEFT_FOREARM("leftForeArm", RobotSide.LEFT, 0.0, 1),
   RIGHT_FOREARM("rightForeArm", RobotSide.RIGHT, 0.0, 1),
   CHEST("chest", null,0.0, 10);

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

   public static VRTrackedSegmentType getHandEnum(RobotSide side)
   {
      if(side == RobotSide.LEFT)
         return VRTrackedSegmentType.LEFT_HAND;
      else
         return VRTrackedSegmentType.RIGHT_HAND;
   }
}