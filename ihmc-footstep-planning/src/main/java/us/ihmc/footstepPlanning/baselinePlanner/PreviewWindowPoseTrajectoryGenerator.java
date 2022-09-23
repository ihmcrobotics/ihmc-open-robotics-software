package us.ihmc.footstepPlanning.baselinePlanner;

import boofcv.concurrency.IntOperatorTask;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
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

   private double MAX_X_VELOCITY;
   private double MAX_Y_VELOCITY;
   private double MAX_YAW_VELOCITY;

   private final double epsilion = 1e-5;

   public PreviewWindowPoseTrajectoryGenerator(ReferenceFrame frame, int windowSize, double dt, double maxVelocityX, double maxVelocityY, double maxVelocityYaw)
   {
      this(frame, windowSize, dt);
      MAX_X_VELOCITY = maxVelocityX;
      MAX_Y_VELOCITY = maxVelocityY;
      MAX_YAW_VELOCITY = maxVelocityYaw;
   }

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

   // Note: append to trajectory from desired pose
   public void append(double dt, FramePose3D framePose3D)
   {
      int prevIdx = headIdx;
      headIdx = (headIdx + 1) % windowSize;
      FramePose3D prevPose = poses[prevIdx];
      FramePose3D headPose = poses[headIdx];

      // translation
      Vector3D distanceVector = new Vector3D();

      // holds position of gizmo
      FramePose3D tempFramePose = new FramePose3D();
      distanceVector.set(framePose3D.getPosition());
      distanceVector.sub(prevPose.getPosition());

      Vector3D distanceVectorInPrevReferenceFrame = new Vector3D();
      prevPose.inverseTransform(distanceVector, distanceVector);

      // z-rotation
      double yaw = EuclidCoreTools.angleDifferenceMinusPiToPi(framePose3D.getYaw(), prevPose.getYaw());
      double yawdot = yaw / dt;

      if (Math.abs(yawdot) > MAX_YAW_VELOCITY)
      {
         yawdot = Math.signum(yawdot) * MAX_YAW_VELOCITY;
      }
      if (Math.abs(distanceVector.getX() / dt) > MAX_X_VELOCITY)
      {
         distanceVector.setX(Math.signum(distanceVector.getX()) * MAX_X_VELOCITY * dt);
      }
      if (Math.abs(distanceVector.getY() / dt) > MAX_Y_VELOCITY)
      {
         distanceVector.setY(Math.signum(distanceVector.getY()) * MAX_Y_VELOCITY * dt);
      }

      headPose.set(poses[prevIdx]);
      //      distanceVector.scale(dt);
      headPose.getPosition().add(distanceVector);
      headPose.appendYawRotation(yawdot * dt);
   }

   // Note: appends velocity vector
   public void append(double deltaTime, double xdot, double ydot, double yawdot)
   {
      int prevIdx = headIdx;
      headIdx = (headIdx + 1) % windowSize;

      FramePose3D headPose = poses[headIdx];
      headPose.set(poses[prevIdx]); // copy pose from prev step
      headPose.appendTranslation(xdot * deltaTime, ydot * deltaTime, 0.0);
      headPose.appendYawRotation(yawdot * deltaTime);
   }

   // Note: (original) appends velocity vector
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

   public FramePose3D[] getPoses()
   {
      return poses;
   }
}