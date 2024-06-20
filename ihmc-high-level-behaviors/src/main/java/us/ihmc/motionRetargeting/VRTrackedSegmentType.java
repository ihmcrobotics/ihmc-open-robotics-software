package us.ihmc.motionRetargeting;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;

public enum VRTrackedSegmentType
{
   // Hands defaults are 20 and 1. Reduce the orientation to 0.25 for the nub forearms
   LEFT_HAND("Left Hand", RobotSide.LEFT, 20.0, 0.25),
   RIGHT_HAND("Right Hand", RobotSide.RIGHT, 20.0, 0.25),
   LEFT_WRIST("Left Wrist", RobotSide.LEFT, 0.0, 1.0),
   RIGHT_WRIST("Right Wrist", RobotSide.RIGHT, 0.0, 1.0),
   CHEST("Chest", null, 0.0, 10.0, 1.5, 1.5),
   WAIST("Waist", null, 0.0, 10.0, 50.0, 10.0, 10.0, 1.5, 1.5),
   LEFT_ANKLE("Left Ankle", RobotSide.LEFT, 0.0, 0.0),
   RIGHT_ANKLE("Right Ankle", RobotSide.RIGHT, 0.0, 0.0);

   private final String segmentName;
   private final RobotSide robotSide;
   private final double linearRateLimitation;
   private final double angularRateLimitation;
   private final Vector3D positionWeight = new Vector3D();
   private final Vector3D orientationWeight = new Vector3D();

   VRTrackedSegmentType(String segmentName,
                        RobotSide robotSide,
                        double positionXYWeight,
                        double positionZWeight,
                        double orientationXWeight,
                        double orientationYWeight,
                        double orientationZWeight,
                        double linearRateLimitation,
                        double angularRateLimitation)
   {
      this.segmentName = segmentName;
      this.robotSide = robotSide;
      this.linearRateLimitation = linearRateLimitation;
      this.angularRateLimitation = angularRateLimitation;
      positionWeight.set(positionXYWeight, positionXYWeight, positionZWeight);
      orientationWeight.set(orientationXWeight, orientationYWeight, orientationZWeight);
   }

   VRTrackedSegmentType(String segmentName, RobotSide robotSide, double positionWeight, double orientationWeight)
   {
      this(segmentName, robotSide, positionWeight, orientationWeight, -1.0, -1.0);
   }
   VRTrackedSegmentType(String segmentName, RobotSide robotSide, double positionWeight, double orientationWeight, double linearRateLimitation, double angularRateLimitation)
   {
      this(segmentName, robotSide, positionWeight, positionWeight, orientationWeight, orientationWeight, orientationWeight, linearRateLimitation, angularRateLimitation);
   }

   public String getSegmentName()
   {
      return segmentName;
   }

   public RobotSide getSegmentSide()
   {
      return robotSide;
   }

   public Vector3D getPositionWeight()
   {
      return positionWeight;
   }

   public Vector3D getOrientationWeight()
   {
      return orientationWeight;
   }

   public double getLinearRateLimitation()
   {
      return linearRateLimitation;
   }

   public double getAngularRateLimitation()
   {
      return angularRateLimitation;
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