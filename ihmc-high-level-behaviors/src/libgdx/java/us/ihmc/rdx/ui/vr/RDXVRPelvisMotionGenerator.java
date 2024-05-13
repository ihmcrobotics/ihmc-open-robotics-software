package us.ihmc.rdx.ui.vr;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.vr.RDXVRTracker;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class RDXVRPelvisMotionGenerator
{
   private static final VRTrackedSegmentType segmentType = VRTrackedSegmentType.WAIST;
   private final RDXVRTracker waistTracker;
   private RigidBodyTransform initialWaistTrackerTransformToWorld;
   private MutableReferenceFrame waistTrackerFrame;
   private MutableReferenceFrame pelvisReferenceFrame;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RetargetingParameters retargetingParameters;
   private final RigidBodyTransform initialPelvisTransformToWorld = new RigidBodyTransform();

   public RDXVRPelvisMotionGenerator(ROS2SyncedRobotModel syncedRobot, RDXVRTracker waistTracker, RetargetingParameters retargetingParameters)
   {
      this.syncedRobot = syncedRobot;
      this.waistTracker = waistTracker;
      this.retargetingParameters = retargetingParameters;
   }

   public void update()
   {
      if (waistTrackerFrame == null)
      {
         initialPelvisTransformToWorld.set(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
         waistTrackerFrame = new MutableReferenceFrame(waistTracker.getXForwardZUpTrackerFrame());
         waistTrackerFrame.getTransformToParent().appendOrientation(retargetingParameters.getYawPitchRollFromTracker(segmentType));
         waistTrackerFrame.getReferenceFrame().update();

         initialWaistTrackerTransformToWorld = new RigidBodyTransform(waistTrackerFrame.getReferenceFrame().getTransformToWorldFrame());
      }
      // Calculate the variation of the tracker's frame from its initial value
      RigidBodyTransform waistTrackerVariationFromInitialValue = new RigidBodyTransform(waistTrackerFrame.getReferenceFrame()
                                                                                                         .getTransformToWorldFrame());
      initialWaistTrackerTransformToWorld.inverseTransform(waistTrackerVariationFromInitialValue);
      double scalingRobotHumanWaistHeight = initialPelvisTransformToWorld.getTranslationZ() / initialWaistTrackerTransformToWorld.getTranslationZ();
      waistTrackerVariationFromInitialValue.getTranslation()
                                           .setZ(scalingRobotHumanWaistHeight * waistTrackerVariationFromInitialValue.getTranslationZ());

      // Concatenate the initial pelvis transform with the variation
      RigidBodyTransform combinedTransformToWorld = new RigidBodyTransform(initialPelvisTransformToWorld);
      combinedTransformToWorld.multiply(waistTrackerVariationFromInitialValue);
      FramePose3D combinedFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), combinedTransformToWorld);
      combinedFramePose.changeFrame(waistTrackerFrame.getReferenceFrame());

      pelvisReferenceFrame = new MutableReferenceFrame(waistTrackerFrame.getReferenceFrame());
      pelvisReferenceFrame.update(rigidBodyTransform -> rigidBodyTransform.set(new RigidBodyTransform(combinedFramePose.getRotation(),
                                                                                                      combinedFramePose.getTranslation())));
   }

   public MutableReferenceFrame getPelvisReferenceFrame()
   {
      return pelvisReferenceFrame;
   }
}
