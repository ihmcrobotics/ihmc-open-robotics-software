package us.ihmc.motionRetargeting;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public abstract class RetargetingParameters
{

   public double getPelvisHeightExtendedLegs()
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
