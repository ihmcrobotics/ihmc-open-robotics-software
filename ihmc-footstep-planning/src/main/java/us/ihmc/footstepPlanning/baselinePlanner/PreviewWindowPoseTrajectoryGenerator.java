package us.ihmc.footstepPlanning.baselinePlanner;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;

public class PreviewWindowPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final int windowSize;
   private final double dt;

   private int headIdx;
   private final FramePose3D[] poses;
   private final FrameVector3D[] linearVelocities;
   private final FrameVector3D[] angularVelocities;

   private final FramePose3D currentPose;
   private final FrameVector3D currentLinearVelocity;
   private final FrameVector3D currentAngularVelocity;

   private final FrameVector3DReadOnly zeroVector;

   public PreviewWindowPoseTrajectoryGenerator(ReferenceFrame frame, int windowSize, double dt)
   {
      this.windowSize = windowSize;
      this.dt = dt;
      this.headIdx = 0;

      zeroVector = new FrameVector3D(frame);

      this.poses = new FramePose3D[windowSize];
      this.linearVelocities = new FrameVector3D[windowSize];
      this.angularVelocities = new FrameVector3D[windowSize];
      for (int i = 0; i < poses.length; i++)
      {
         poses[i] = new FramePose3D();
         linearVelocities[i] = new FrameVector3D();
         angularVelocities[i] = new FrameVector3D();
      }

      this.currentPose = new FramePose3D(frame);
      this.currentLinearVelocity = new FrameVector3D(frame);
      this.currentAngularVelocity = new FrameVector3D(frame);
   }

   @Override
   public void initialize()
   {
      // do nothing
   }

   public void reset(FramePose3DReadOnly initialPose)
   {
      for (int i = 0; i < poses.length; i++)
      {
         poses[i].set(initialPose);
         linearVelocities[i].setToZero();
         angularVelocities[i].setToZero();
      }

      currentPose.setToZero();
      currentLinearVelocity.setToZero();
      currentAngularVelocity.setToZero();
   }

   public void append(double xdot, double ydot, double yawdot)
   {
      int prevIdx = headIdx;
      headIdx = (headIdx + 1) % windowSize;

      FramePose3D headPose = poses[headIdx];
      headPose.set(poses[prevIdx]); // copy pose from prev step
      headPose.appendTranslation(xdot * dt, ydot * dt, 0.0);
      headPose.appendYawRotation(yawdot * dt);

      linearVelocities[headIdx].set(xdot, ydot, 0.0);
      headPose.transform(linearVelocities[headIdx]);
      angularVelocities[headIdx].set(0.0, 0.0, yawdot);
   }

   private int getLookaheadIndex(int lookahead)
   {
      // wrap(tail + lookahead)
      int tailIdx = (headIdx + 1) % windowSize;
      int lookaheadIdx = (tailIdx + lookahead) % windowSize;
      if (lookaheadIdx < 0 || lookaheadIdx >= windowSize)
         throw new IndexOutOfBoundsException("trajectory index out of bounds: 0 <= " + lookahead + " < " + windowSize);

      return lookaheadIdx;
   }

   public int getWindowSize()
   {
      return windowSize;
   }

   @Override
   public void compute(double time)
   {
      int floorIdx = (int) Math.min(windowSize - 2, Math.max(0, Math.floor(time / dt)));
      int ceilIdx = Math.min(windowSize - 1, floorIdx + 1);

      double floorTime = floorIdx * dt;
      double ceilTime = ceilIdx * dt;

      floorIdx = getLookaheadIndex(floorIdx);
      ceilIdx = getLookaheadIndex(ceilIdx);

      double alpha = 0.0;
      if (floorIdx != ceilIdx)
         alpha = (time - floorTime) / (ceilTime - floorTime);

      FramePose3DReadOnly floorPose = poses[floorIdx];
      FramePose3DReadOnly ceilPose = poses[ceilIdx];
      currentPose.interpolate(floorPose, ceilPose, alpha);

      FrameVector3DReadOnly floorLinearVelocity = linearVelocities[floorIdx];
      FrameVector3DReadOnly ceilLinearVelocity = linearVelocities[ceilIdx];
      currentLinearVelocity.interpolate(floorLinearVelocity, ceilLinearVelocity, alpha);

      FrameVector3DReadOnly floorAngularVelocity = angularVelocities[floorIdx];
      FrameVector3DReadOnly ceilAngularVelocity = angularVelocities[ceilIdx];
      currentAngularVelocity.interpolate(floorAngularVelocity, ceilAngularVelocity, alpha);
   }

   @Override
   public FramePose3DReadOnly getPose()
   {
      return currentPose;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return currentLinearVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return zeroVector;
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return currentAngularVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAngularAcceleration()
   {
      return zeroVector;
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}