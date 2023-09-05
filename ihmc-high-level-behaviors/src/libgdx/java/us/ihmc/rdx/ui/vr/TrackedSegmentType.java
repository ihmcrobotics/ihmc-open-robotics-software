package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;

public enum TrackedSegmentType
{
   LEFT_HAND("leftHand", RobotSide.LEFT, new YawPitchRoll(), Double.NaN, Double.NaN),
   RIGHT_HAND("rightHand", RobotSide.RIGHT, new YawPitchRoll(), Double.NaN, Double.NaN),
   LEFT_FOREARM("leftForeArm", RobotSide.LEFT, new YawPitchRoll(-Math.PI / 2.0, 0.0, 0.0), 0, 0.1),
   RIGHT_FOREARM("rightForeArm", RobotSide.RIGHT, new YawPitchRoll(Math.PI / 2.0, 0.0, 0.0), 0, 0.1),
   CHEST("chest", null, new YawPitchRoll(), 0, 10);

   private String segmentName;
   private RobotSide robotSide;
   private YawPitchRoll trackerToRigidBodyRotation;
   private double positionWeight;
   private double orientationWeight;

   TrackedSegmentType(String segmentName, RobotSide robotSide, YawPitchRoll trackerToRigidBodyRotation, double positionWeight, double orientationWeight)
   {
      this.segmentName = segmentName;
      this.robotSide = robotSide;
      this.trackerToRigidBodyRotation = trackerToRigidBodyRotation;
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

   public YawPitchRoll getTrackerToRigidBodyRotation()
   {
      return trackerToRigidBodyRotation;
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
