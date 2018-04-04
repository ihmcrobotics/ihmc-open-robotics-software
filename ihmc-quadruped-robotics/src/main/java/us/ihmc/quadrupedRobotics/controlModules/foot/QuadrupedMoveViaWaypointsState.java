package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // Yo variables
   private final YoDouble robotTime;

   // SoleWaypoint variables
   private final MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame soleFrame;
   private final QuadrupedSoleWaypointList quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();

   private final QuadrupedFootControlModuleParameters parameters;

   private final FrameVector3D initialSoleForces = new FrameVector3D();

   private final VirtualForceCommand virtualForceCommand = new VirtualForceCommand();

   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootVelocity = new FrameVector3D();

   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private double taskStartTime;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      this.bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      this.soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      this.parameters = controllerToolbox.getFootControlModuleParameters();
      robotTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      // Create waypoint trajectory
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       bodyFrame, registry);

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
   }

   public void handleWaypointList(QuadrupedSoleWaypointList quadrupedSoleWaypointList)
   {
      this.quadrupedSoleWaypointList.set(quadrupedSoleWaypointList);
   }

   public void initialize(boolean useInitialSoleForceAsFeedforwardTerm)
   {
      if (useInitialSoleForceAsFeedforwardTerm)
      {
         initialSoleForces.setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSoleVirtualForce(robotQuadrant));
         initialSoleForces.changeFrame(worldFrame);
      }
      else
      {
         initialSoleForces.setToZero(bodyFrame);
         initialSoleForces.changeFrame(worldFrame);
      }

      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.getFootContactState(robotQuadrant).clear();
   }

   @Override
   public void doAction(double timeInState)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
      {
         soleForceCommand.setToZero();

         if (waypointCallback != null)
            waypointCallback.isDoneMoving(true);

         desiredFootPosition.setToZero(soleFrame);
         desiredFootPosition.changeFrame(worldFrame);
         desiredFootVelocity.setToZero(worldFrame);

         virtualForceCommand.setLinearForce(soleFrame, soleForceCommand);
      }
      else
      {
         quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
         quadrupedWaypointsPositionTrajectoryGenerator.getPosition(desiredFootPosition);
         desiredFootVelocity.setToZero();

         feedbackControlCommand.set(desiredFootPosition, desiredFootVelocity);
         feedbackControlCommand.setFeedForwardAction(initialSoleForces);
         feedbackControlCommand.setGains(parameters.getSolePositionGains());

         if (waypointCallback != null)
            waypointCallback.isDoneMoving(false);
      }
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;
      return currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime() ? QuadrupedFootControlModule.FootEvent.TIMEOUT : null;
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      for (int i = 0; i < quadrupedSoleWaypointList.size(); ++i)
      {
         quadrupedWaypointsPositionTrajectoryGenerator
               .appendWaypoint(quadrupedSoleWaypointList.get(i).getTime(), quadrupedSoleWaypointList.get(i).getPosition(),
                               quadrupedSoleWaypointList.get(i).getVelocity());
      }
      if (quadrupedSoleWaypointList.size() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
         return virtualForceCommand;
      else
         return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
         return null;
      else
         return feedbackControlCommand;
   }
}
