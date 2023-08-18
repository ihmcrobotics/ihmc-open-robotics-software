package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;

public enum TrackedSegmentType
{
   LEFT_FOREARM("leftForeArm", RobotSide.LEFT, new YawPitchRoll(-Math.PI / 2.0, 0.0, 0.0), 0, 0.1),
   RIGHT_FOREARM("rightForeArm", RobotSide.RIGHT, new YawPitchRoll(Math.PI / 2.0, 0.0, 0.0), 0, 0.1),
   CHEST("chest", null, new YawPitchRoll(), 0, 10);

   String segmentName;
   RobotSide robotSide;
   YawPitchRoll trackerToRigidBodyRotation;
   double positionWeight;
   double orientationWeight;

   TrackedSegmentType(String segmentName, RobotSide robotSide, YawPitchRoll trackerToRigidBodyRotation, double positionWeight, double orientationWeight)
   {
      this.segmentName = segmentName;
      this.robotSide = robotSide;
      this.trackerToRigidBodyRotation = trackerToRigidBodyRotation;
      this.positionWeight = positionWeight;
      this.orientationWeight = orientationWeight;
   }
}
