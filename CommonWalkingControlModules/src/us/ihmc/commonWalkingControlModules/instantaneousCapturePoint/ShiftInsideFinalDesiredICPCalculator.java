package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;


import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;

/**
 * Very simple FinalDesiredICPCalculator that just puts the final desired ICP 4cm or so inside the transfer to foot.
 *
 */
public class ShiftInsideFinalDesiredICPCalculator implements FinalDesiredICPCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final double extraX;
   private final double extraY;

   public ShiftInsideFinalDesiredICPCalculator(YoVariableRegistry parentRegistry, double extraX, double extraY)
   {
      parentRegistry.addChild(registry);
      this.extraX = extraX;
      this.extraY = extraY;
   }

   private FramePoint2d finalDesiredICP;

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      finalDesiredICP = getFinalDesiredICPForWalking(transferToAndNextFootstepsData);
   }

   public FramePoint2d getFinalDesiredICP()
   {
      return finalDesiredICP;
   }

   private FramePoint2d getFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();

      FramePose transferToFootstepAnklePose = new FramePose();
      transferToFootstep.getPose(transferToFootstepAnklePose);
      FrameConvexPolygon2d transferToFootPolygonInSoleFrame = transferToAndNextFootstepsData.getTransferToFootPolygonInSoleFrame();
      RobotSide transferToSide = transferToAndNextFootstepsData.getTransferToSide();

      RigidBodyTransform footstepAnkleToWorldTransform = new RigidBodyTransform();
      getTransformFromPoseToWorld(footstepAnkleToWorldTransform, transferToFootstepAnklePose);

      ReferenceFrame footBodyFrame = transferToFootstep.getBody().getParentJoint().getFrameAfterJoint();
      ReferenceFrame footPlaneFrame = transferToFootstep.getSoleReferenceFrame();
      RigidBodyTransform ankleToSoleTransform = footPlaneFrame.getTransformToDesiredFrame(footBodyFrame);

      FramePoint2d centroid2d = transferToFootPolygonInSoleFrame.getCentroidCopy();
      FramePoint centroid = centroid2d.toFramePoint();
      centroid.changeFrameUsingTransform(null, ankleToSoleTransform);
      centroid.changeFrameUsingTransform(worldFrame, footstepAnkleToWorldTransform);

      FramePoint pointOffsetFromCentroid = new FramePoint(centroid);

      double extraY = transferToSide.negateIfLeftSide(this.extraY);
      FrameVector offset = new FrameVector(null, extraX, extraY, 0.0);
      offset.changeFrameUsingTransform(worldFrame, footstepAnkleToWorldTransform);

      pointOffsetFromCentroid.changeFrame(offset.getReferenceFrame());
      pointOffsetFromCentroid.add(offset);

      FramePoint2d ret = pointOffsetFromCentroid.toFramePoint2d();

      return ret;
   }


   private static void getTransformFromPoseToWorld(RigidBodyTransform poseToWorldTransformToPack, FramePose framePose)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      framePose.changeFrame(worldFrame);
      framePose.getPose(poseToWorldTransformToPack);
   }
}
