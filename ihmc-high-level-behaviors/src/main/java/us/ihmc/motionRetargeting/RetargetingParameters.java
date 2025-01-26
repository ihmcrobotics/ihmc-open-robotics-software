package us.ihmc.motionRetargeting;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class RetargetingParameters
{
   public float getHomePoint(String jointName)
   {
      return 0.0f;
   }

   public float getArmHomePoint(RobotSide robotSide, ArmJointName jointName)
   {
      return 0.0f;
   }

   public Vector3D getPositionWeight(VRTrackedSegmentType tracker)
   {
      return new Vector3D(-1.0, -1.0, -1.0); // default values are used
   }

   public Vector3D getOrientationWeight(VRTrackedSegmentType tracker)
   {
      return new Vector3D(-1.0, -1.0, -1.0); // default values are used
   }

   public double getLinearRateLimitation(VRTrackedSegmentType tracker)
   {
      return -1.0; // default value is used
   }

   public double getAngularRateLimitation(VRTrackedSegmentType tracker)
   {
      return -1.0; // default value is used
   }

   public double getPelvisHeightExtendedLegs()
   {
      return 1.0;
   }

   public double getArmLength()
   {
      return 1.0;
   }

   public Point3D getTranslationFromTracker(VRTrackedSegmentType tracker)
   {
      return new Point3D();
   }

   public YawPitchRoll getYawPitchRollFromTracker(VRTrackedSegmentType tracker)
   {
      return new YawPitchRoll();
   }
}
