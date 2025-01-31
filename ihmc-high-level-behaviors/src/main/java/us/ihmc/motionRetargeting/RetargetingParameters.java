package us.ihmc.motionRetargeting;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class RetargetingParameters
{
   protected final SideDependentList<FramePoint3DReadOnly> handControlFrameOffsetsInBodyFrame = new SideDependentList<>();

   public RetargetingParameters()
   {
      this(null);
   }

   public RetargetingParameters(FullHumanoidRobotModel fullRobotModel)
   {
      if (fullRobotModel != null)
         setHandControlFrameOffsets(fullRobotModel);
   }

   private void setHandControlFrameOffsets(FullHumanoidRobotModel fullRobotModel)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint3D handControlFrameOffset = new FramePoint3D(fullRobotModel.getHandControlFrame(robotSide));
         handControlFrameOffset.changeFrame(fullRobotModel.getHand(robotSide).getBodyFixedFrame());
         handControlFrameOffsetsInBodyFrame.put(robotSide, handControlFrameOffset);
      }
   }

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

   public Point3D getControlFrameOffsetInBodyFrame(VRTrackedSegmentType tracker)
   {
      Point3D translationFromTracker = new Point3D();

      if (handControlFrameOffsetsInBodyFrame.isEmpty())
         return translationFromTracker;

      if (tracker == VRTrackedSegmentType.LEFT_HAND)
      {
         translationFromTracker.set(handControlFrameOffsetsInBodyFrame.get(RobotSide.LEFT));
      }
      else if (tracker == VRTrackedSegmentType.RIGHT_HAND)
      {
         translationFromTracker.set(handControlFrameOffsetsInBodyFrame.get(RobotSide.RIGHT));
      }

      return translationFromTracker;
   }

   public YawPitchRoll getControlFrameOrientationInBodyFrame(VRTrackedSegmentType tracker)
   {
      return new YawPitchRoll();
   }
}
