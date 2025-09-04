package us.ihmc.motionRetargeting;

import us.ihmc.robotics.robotSide.RobotSide;

public enum VRTrackedSegmentType
{
   LEFT_HAND("Left Hand", RobotSide.LEFT),
   RIGHT_HAND("Right Hand", RobotSide.RIGHT),
   LEFT_WRIST("Left Wrist", RobotSide.LEFT),
   RIGHT_WRIST("Right Wrist", RobotSide.RIGHT),
   HEAD("Head", null),
   CHEST("Chest", null),
   WAIST("Waist", null),
   LEFT_ANKLE("Left Ankle", RobotSide.LEFT),
   RIGHT_ANKLE("Right Ankle", RobotSide.RIGHT);

   private final String segmentName;
   private final RobotSide robotSide;

   public static VRTrackedSegmentType[] TRACKER_TYPES = new VRTrackedSegmentType[] {LEFT_WRIST, RIGHT_WRIST, CHEST, WAIST, LEFT_ANKLE, RIGHT_ANKLE};
   public static VRTrackedSegmentType[] CONTROLLER_TYPES = new VRTrackedSegmentType[] {LEFT_HAND, RIGHT_HAND};
   public static VRTrackedSegmentType[] FOOT_TYPES = new VRTrackedSegmentType[] {LEFT_ANKLE, RIGHT_ANKLE};

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

   public boolean isWristRelated()
   {
      return segmentName.contains("Wrist");
   }

   public boolean isHandRelated()
   {
      return segmentName.contains("Hand");
   }
}