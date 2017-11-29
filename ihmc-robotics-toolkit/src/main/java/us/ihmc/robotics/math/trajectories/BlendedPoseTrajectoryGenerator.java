package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BlendedPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final BlendedPositionTrajectoryGenerator blendedPositionTrajectory;
   private final BlendedOrientationTrajectoryGenerator blendedOrientationTrajectory;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   public BlendedPoseTrajectoryGenerator(String prefix, PoseTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame, YoVariableRegistry parentRegistry)
   {
      this.blendedPositionTrajectory = new BlendedPositionTrajectoryGenerator(prefix + "Position", trajectory, trajectoryFrame, parentRegistry);
      this.blendedOrientationTrajectory = new BlendedOrientationTrajectoryGenerator(prefix + "Orientation", trajectory, trajectoryFrame, parentRegistry);
   }

   public void clear()
   {
      blendedPositionTrajectory.clear();
      blendedOrientationTrajectory.clear();
   }

   public void clearInitialConstraint()
   {
      blendedPositionTrajectory.clearInitialConstraint();
      blendedOrientationTrajectory.clearInitialConstraint();
   }

   public void clearFinalConstraint()
   {
      blendedPositionTrajectory.clearFinalConstraint();
      blendedOrientationTrajectory.clearFinalConstraint();
   }

   public void blendInitialConstraint(FramePose initialPose, double initialTime, double blendDuration)
   {
      initialPose.getPositionIncludingFrame(tempPosition);
      initialPose.getOrientationIncludingFrame(tempOrientation);
      blendedPositionTrajectory.blendInitialConstraint(tempPosition, initialTime, blendDuration);
      blendedOrientationTrajectory.blendInitialConstraint(tempOrientation, initialTime, blendDuration);
   }

   public void blendInitialConstraint(FramePose initialPose, Twist initialTwist, double initialTime, double blendDuration)
   {
      initialPose.getPositionIncludingFrame(tempPosition);
      initialPose.getOrientationIncludingFrame(tempOrientation);
      initialTwist.getLinearPart(tempVelocity);
      initialTwist.getAngularPart(tempAngularVelocity);
      blendedPositionTrajectory.blendInitialConstraint(tempPosition, tempVelocity, initialTime, blendDuration);
      blendedOrientationTrajectory.blendInitialConstraint(tempOrientation, tempAngularVelocity, initialTime, blendDuration);
   }

   public void blendFinalConstraint(FramePose finalPose, double finalTime, double blendDuration)
   {
      finalPose.getPositionIncludingFrame(tempPosition);
      finalPose.getOrientationIncludingFrame(tempOrientation);
      blendedPositionTrajectory.blendFinalConstraint(tempPosition, finalTime, blendDuration);
      blendedOrientationTrajectory.blendFinalConstraint(tempOrientation, finalTime, blendDuration);
   }

   public void blendFinalConstraint(FramePose finalPose, Twist finalTwist, double finalTime, double blendDuration)
   {
      finalPose.getPositionIncludingFrame(tempPosition);
      finalPose.getOrientationIncludingFrame(tempOrientation);
      finalTwist.getLinearPart(tempVelocity);
      finalTwist.getAngularPart(tempAngularVelocity);
      blendedPositionTrajectory.blendFinalConstraint(tempPosition, tempVelocity, finalTime, blendDuration);
      blendedOrientationTrajectory.blendFinalConstraint(tempOrientation, tempAngularVelocity, finalTime, blendDuration);
   }

   @Override
   public void getPose(FramePose framePoseToPack)
   {
      blendedPositionTrajectory.getPosition(tempPosition);
      blendedOrientationTrajectory.getOrientation(tempOrientation);
      framePoseToPack.setPoseIncludingFrame(tempPosition, tempOrientation);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      blendedPositionTrajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      blendedPositionTrajectory.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      blendedPositionTrajectory.getAcceleration(accelerationToPack);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      blendedOrientationTrajectory.getOrientation(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      blendedOrientationTrajectory.getAngularVelocity(angularVelocityToPack);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      blendedOrientationTrajectory.getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      blendedPositionTrajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      blendedPositionTrajectory.hideVisualization();
   }

   @Override
   public void initialize()
   {
      blendedPositionTrajectory.initialize();
      blendedOrientationTrajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      blendedPositionTrajectory.compute(time);
      blendedOrientationTrajectory.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return blendedPositionTrajectory.isDone() && blendedOrientationTrajectory.isDone();
   }
}
