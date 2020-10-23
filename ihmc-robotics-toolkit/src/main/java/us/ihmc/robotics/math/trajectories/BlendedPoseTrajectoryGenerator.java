package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BlendedPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final BlendedPositionTrajectoryGenerator blendedPositionTrajectory;
   private final BlendedOrientationTrajectoryGenerator blendedOrientationTrajectory;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   private final PoseTrajectoryGenerator trajectory;

   public BlendedPoseTrajectoryGenerator(String prefix, PoseTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame, YoRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.blendedPositionTrajectory = new BlendedPositionTrajectoryGenerator(prefix + "Position", trajectory, trajectoryFrame, parentRegistry);
      this.blendedOrientationTrajectory = new BlendedOrientationTrajectoryGenerator(prefix + "Orientation", trajectory, trajectoryFrame, parentRegistry);
   }

   public BlendedPositionTrajectoryGenerator getPositionTrajectoryGenerator()
   {
      return blendedPositionTrajectory;
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

   public void blendInitialConstraint(FramePose3DReadOnly initialPose, double initialTime, double blendDuration)
   {
      blendedPositionTrajectory.blendInitialConstraint(initialPose.getPosition(), initialTime, blendDuration);
      blendedOrientationTrajectory.blendInitialConstraint(initialPose.getOrientation(), initialTime, blendDuration);
   }

   public void blendInitialConstraint(FramePose3DReadOnly initialPose, TwistReadOnly initialTwist, double initialTime, double blendDuration)
   {
      tempVelocity.setIncludingFrame(initialTwist.getLinearPart());
      tempAngularVelocity.setIncludingFrame(initialTwist.getAngularPart());
      blendedPositionTrajectory.blendInitialConstraint(initialPose.getPosition(), tempVelocity, initialTime, blendDuration);
      blendedOrientationTrajectory.blendInitialConstraint(initialPose.getOrientation(), tempAngularVelocity, initialTime, blendDuration);
   }

   public void blendFinalConstraint(FramePose3DReadOnly finalPose, double finalTime, double blendDuration)
   {
      blendedPositionTrajectory.blendFinalConstraint(finalPose.getPosition(), finalTime, blendDuration);
      blendedOrientationTrajectory.blendFinalConstraint(finalPose.getOrientation(), finalTime, blendDuration);
   }

   public void blendFinalConstraint(FramePose3DReadOnly finalPose, TwistReadOnly finalTwist, double finalTime, double blendDuration)
   {
      tempVelocity.setIncludingFrame(finalTwist.getLinearPart());
      tempAngularVelocity.setIncludingFrame(finalTwist.getAngularPart());
      blendedPositionTrajectory.blendFinalConstraint(finalPose.getPosition(), tempVelocity, finalTime, blendDuration);
      blendedOrientationTrajectory.blendFinalConstraint(finalPose.getOrientation(), tempAngularVelocity, finalTime, blendDuration);
   }

   public void initializeTrajectory()
   {
      trajectory.initialize();
   }


   @Override
   public void getPose(FramePose3D framePoseToPack)
   {
      blendedPositionTrajectory.getPosition(tempPosition);
      blendedOrientationTrajectory.getOrientation(tempOrientation);
      framePoseToPack.setIncludingFrame(tempPosition, tempOrientation);
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
