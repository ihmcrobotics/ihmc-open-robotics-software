package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Reduced order robot model consisting of the centroidal and contact states.
 */
public class ReducedOrderRobotModel
{
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();

   private final PoseReferenceFrame midFeetZUpFrame = new PoseReferenceFrame("midFeetZUpFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final SideDependentList<FramePose3D> feetPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final SideDependentList<FramePoint3D> handPositions = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final SideDependentList<FrameVector3D> handVelocities = new SideDependentList<>(new FrameVector3D(), new FrameVector3D());

   public void set(ReducedOrderRobotModel other)
   {
      this.comPosition.set(other.comPosition);
      this.comVelocity.set(other.comVelocity);

      for (RobotSide robotSide : RobotSide.values)
      {
         feetPoses.get(robotSide).set(other.feetPoses.get(robotSide));
         handPositions.get(robotSide).set(other.handPositions.get(robotSide));
         handVelocities.get(robotSide).set(other.handVelocities.get(robotSide));
      }

      updateMidFeetZUp();
   }

   private void updateMidFeetZUp()
   {
      midFeetZUpPose.interpolate(feetPoses.get(RobotSide.LEFT), feetPoses.get(RobotSide.RIGHT), 0.5);
      midFeetZUpPose.getOrientation().setToYawOrientation(midFeetZUpPose.getOrientation().getYaw());
      midFeetZUpFrame.setPoseAndUpdate(midFeetZUpPose);
   }

   public FramePoint3DReadOnly getHandPosition(RobotSide robotSide)
   {
      return handPositions.get(robotSide);
   }

   public FramePoint3DReadOnly getComPosition()
   {
      return comPosition;
   }

   public FrameVector3DReadOnly getComVelocity()
   {
      return comVelocity;
   }

   public PoseReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUpFrame;
   }

   public void initialize(FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, FrameVector3DReadOnly comVelocity)
   {
      this.comPosition.setToZero(referenceFrames.getCenterOfMassFrame());
      this.comVelocity.setIncludingFrame(comVelocity);

      this.comPosition.changeFrame(ReferenceFrame.getWorldFrame());
      this.comVelocity.changeFrame(ReferenceFrame.getWorldFrame());

      for (RobotSide robotSide : RobotSide.values)
      {
         this.feetPoses.get(robotSide).setToZero(referenceFrames.getSoleFrame(robotSide));
         this.handPositions.get(robotSide).setToZero(fullRobotModel.getHandControlFrame(robotSide));
         this.handVelocities.get(robotSide).setToZero();

         this.feetPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
         this.handPositions.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
         this.handVelocities.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
      }

      updateMidFeetZUp();
   }
}
