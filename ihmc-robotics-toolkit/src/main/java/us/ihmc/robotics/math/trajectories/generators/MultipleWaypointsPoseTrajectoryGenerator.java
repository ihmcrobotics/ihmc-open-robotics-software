package us.ihmc.robotics.math.trajectories.generators;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MultipleWaypointsPoseTrajectoryGenerator implements FixedFramePoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectory;
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory;

   private ReferenceFrame activeFrame;

   private final FramePose3DReadOnly pose;

   public MultipleWaypointsPoseTrajectoryGenerator(String namePrefix, int maxNumberOfWaypoints, YoRegistry parentRegistry)
   {
      positionTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix, maxNumberOfWaypoints, worldFrame, parentRegistry);
      orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, maxNumberOfWaypoints, worldFrame, parentRegistry);
      activeFrame = worldFrame;

      pose = new FramePose3DReadOnly()
      {
         @Override
         public FramePoint3DReadOnly getPosition()
         {
            return positionTrajectory.getPosition();
         }

         @Override
         public FrameQuaternionReadOnly getOrientation()
         {
            return orientationTrajectory.getOrientation();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return activeFrame;
         }
      };
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      positionTrajectory.clear(referenceFrame);
      orientationTrajectory.clear(referenceFrame);
      activeFrame = referenceFrame;
   }

   public MultipleWaypointsPositionTrajectoryGenerator getPositionTrajectory()
   {
      return positionTrajectory;
   }

   public void appendPoseWaypoint(FrameSE3TrajectoryPoint waypoint)
   {
      waypoint.changeFrame(activeFrame);
      positionTrajectory.appendWaypoint(waypoint);
      orientationTrajectory.appendWaypoint(waypoint);
   }

   public void appendPoseWaypoint(double timeAtWaypoint, FramePose3D pose, FrameVector3D linearVelocity, FrameVector3D angularVelocity)
   {
      pose.changeFrame(activeFrame);
      linearVelocity.changeFrame(activeFrame);
      angularVelocity.changeFrame(activeFrame);

      positionTrajectory.appendWaypoint(timeAtWaypoint, pose.getPosition(), linearVelocity);
      orientationTrajectory.appendWaypoint(timeAtWaypoint, pose.getOrientation(), angularVelocity);
   }

   public void appendPositionWaypoint(double timeAtWaypoint, FramePoint3DBasics position, FrameVector3DBasics linearVelocity)
   {
      position.changeFrame(activeFrame);
      linearVelocity.changeFrame(activeFrame);
      positionTrajectory.appendWaypoint(timeAtWaypoint, position, linearVelocity);
   }

   public void appendPositionWaypoint(FrameEuclideanTrajectoryPoint positionWaypoint)
   {
      positionWaypoint.changeFrame(activeFrame);
      positionTrajectory.appendWaypoint(positionWaypoint);
   }

   public void appendOrientationWaypoint(double timeAtWaypoint, FrameQuaternion orientation, FrameVector3D angularVelocity)
   {
      orientation.changeFrame(activeFrame);
      angularVelocity.changeFrame(activeFrame);
      orientationTrajectory.appendWaypoint(timeAtWaypoint, orientation, angularVelocity);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      positionTrajectory.changeFrame(referenceFrame);
      orientationTrajectory.changeFrame(referenceFrame);
      activeFrame = referenceFrame;
   }

   public int getCurrentPositionWaypointIndex()
   {
      return positionTrajectory.getCurrentWaypointIndex();
   }

   @Override
   public void initialize()
   {
      positionTrajectory.initialize();
      orientationTrajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      positionTrajectory.compute(time);
      orientationTrajectory.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return positionTrajectory.isDone() && orientationTrajectory.isDone();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return activeFrame;
   }

   @Override
   public FramePose3DReadOnly getPose()
   {
      return pose;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return positionTrajectory.getVelocity();
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return positionTrajectory.getAcceleration();
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return orientationTrajectory.getAngularVelocity();
   }

   @Override
   public FrameVector3DReadOnly getAngularAcceleration()
   {
      return orientationTrajectory.getAngularAcceleration();
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
