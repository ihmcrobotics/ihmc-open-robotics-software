package us.ihmc.motionRetargeting;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;

public enum VRTrackedSegmentType
{
   // TODO Override parameters in robot dependent class
   // Hands defaults are 20 and 1. Reduce the orientation to 0.25 for the nub forearms
   LEFT_HAND("Left Hand", RobotSide.LEFT),
   RIGHT_HAND("Right Hand", RobotSide.RIGHT),
   LEFT_WRIST("Left Wrist", RobotSide.LEFT),
   RIGHT_WRIST("Right Wrist", RobotSide.RIGHT),
   CHEST("Chest", null),
   WAIST("Waist", null),
   LEFT_ANKLE("Left Ankle", RobotSide.LEFT),
   RIGHT_ANKLE("Right Ankle", RobotSide.RIGHT);

   private final String segmentName;
   private final RobotSide robotSide;

   VRTrackedSegmentType(String segmentName,
                        RobotSide robotSide)
   {
      this.segmentName = segmentName;
      this.robotSide = robotSide;
   }

   public String getSegmentName()
   {
      return segmentName;
   }

   public RobotSide getSegmentSide()
   {
      return robotSide;
   }

   public boolean isFootRelated()
   {
      return segmentName.contains("Ankle") || segmentName.contains("Foot");
   }

   public boolean isHandRelated()
   {
      return segmentName.contains("Hand");
   }

   public static VRTrackedSegmentType[] getTrackerTypes()
   {
      return new VRTrackedSegmentType[] {LEFT_WRIST, RIGHT_WRIST, CHEST, WAIST, LEFT_ANKLE, RIGHT_ANKLE};
   }

   public static VRTrackedSegmentType[] getControllerTypes()
   {
      return new VRTrackedSegmentType[] {LEFT_HAND, RIGHT_HAND};
   }
}