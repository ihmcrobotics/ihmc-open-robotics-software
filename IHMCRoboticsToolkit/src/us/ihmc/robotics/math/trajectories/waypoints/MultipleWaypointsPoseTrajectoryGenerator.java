package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.Collection;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class MultipleWaypointsPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectory;
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory;
   
   private ReferenceFrame activeFrame;
   
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameOrientation desiredOrientation = new FrameOrientation();

   public MultipleWaypointsPoseTrajectoryGenerator(String namePrefix, int maxNumberOfWaypoints, YoVariableRegistry parentRegistry)
   {
      positionTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix, maxNumberOfWaypoints, true, worldFrame, parentRegistry);
      orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, maxNumberOfWaypoints, true, worldFrame, parentRegistry);
      activeFrame = worldFrame;
   }
   
   public void registerFrames(Collection<ReferenceFrame> referenceFrames)
   {
      if (referenceFrames == null)
         return;

      for (ReferenceFrame referenceFrame : referenceFrames)
         registerFrame(referenceFrame);
   }
   
   public void registerFrame(ReferenceFrame referenceFrame)
   {
      positionTrajectory.registerNewTrajectoryFrame(referenceFrame);
      orientationTrajectory.registerNewTrajectoryFrame(referenceFrame);
   }
   
   public void clear(ReferenceFrame referenceFrame)
   {
      positionTrajectory.clear(referenceFrame);
      orientationTrajectory.clear(referenceFrame);
      activeFrame = referenceFrame;
   }
   
   public void appendPoseWaypoint(FrameSE3TrajectoryPoint waypoint)
   {
      waypoint.changeFrame(activeFrame);
      positionTrajectory.appendWaypoint(waypoint);
      orientationTrajectory.appendWaypoint(waypoint);
   }

   public void appendPositionWaypoint(double timeAtWaypoint, FramePoint position, FrameVector linearVelocity)
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
   
   public void appendOrientationWaypoint(double timeAtWaypoint, FrameOrientation orientation, FrameVector angularVelocity)
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
   public void getPosition(FramePoint positionToPack)
   {
      positionTrajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      positionTrajectory.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      positionTrajectory.getAcceleration(accelerationToPack);
   }

   @Override
   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationTrajectory.getOrientation(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      orientationTrajectory.getAngularVelocity(angularVelocityToPack);
   }

   @Override
   public void getAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      orientationTrajectory.getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public void getPose(FramePose framePoseToPack)
   {
      getPosition(desiredPosition);
      getOrientation(desiredOrientation);
      framePoseToPack.setPose(desiredPosition, desiredOrientation);
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
