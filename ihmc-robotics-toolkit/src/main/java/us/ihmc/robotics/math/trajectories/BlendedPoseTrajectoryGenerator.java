package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePoseTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BlendedPoseTrajectoryGenerator implements FixedFramePoseTrajectoryGenerator
{
   private final BlendedWaypointPositionTrajectoryGenerator blendedPositionTrajectory;
   private final BlendedOrientationTrajectoryGenerator blendedOrientationTrajectory;

   private final FixedFramePoseTrajectoryGenerator trajectory;

   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   public BlendedPoseTrajectoryGenerator(String prefix, FixedFramePoseTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame, YoRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.blendedPositionTrajectory = new BlendedWaypointPositionTrajectoryGenerator(prefix + "Position", trajectory, trajectoryFrame, parentRegistry);
      this.blendedOrientationTrajectory = new BlendedOrientationTrajectoryGenerator(prefix + "Orientation", trajectory, trajectoryFrame, parentRegistry);
   }

   public BlendedOrientationTrajectoryGenerator getOrientationTrajectoryGenerator()
   {
      return blendedOrientationTrajectory;
   }

   public BlendedWaypointPositionTrajectoryGenerator getPositionTrajectoryGenerator()
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

   private final FramePose3DReadOnly dummyPose = new FramePose3DReadOnly()
   {
      @Override
      public FramePoint3DReadOnly getPosition()
      {
         return blendedPositionTrajectory.getPosition();
      }

      @Override
      public FrameQuaternionReadOnly getOrientation()
      {
         return blendedOrientationTrajectory.getOrientation();
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return blendedPositionTrajectory.getReferenceFrame();
      }
   };

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return blendedPositionTrajectory.getReferenceFrame();
   }

   @Override
   public FramePose3DReadOnly getPose()
   {
      return dummyPose;
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return blendedPositionTrajectory.getPosition();
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return blendedPositionTrajectory.getVelocity();
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return blendedPositionTrajectory.getAcceleration();
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return blendedOrientationTrajectory.getOrientation();
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return blendedOrientationTrajectory.getAngularVelocity();
   }

   @Override
   public FrameVector3DReadOnly getAngularAcceleration()
   {
      return blendedOrientationTrajectory.getAngularAcceleration();
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
