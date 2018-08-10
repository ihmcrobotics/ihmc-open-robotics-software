package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFootControlModuleParameters;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
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
   private final FrameEuclideanTrajectoryPointList trajectoryPointList = new FrameEuclideanTrajectoryPointList();

   private final QuadrupedFootControlModuleParameters parameters;

   private final VirtualForceCommand virtualForceCommand = new VirtualForceCommand();

   private final QuadrupedControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootVelocity = new FrameVector3D();

   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private double taskStartTime;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      this.bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      ReferenceFrame soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
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

   public void handleWaypointList(FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.set(trajectoryPointList);
   }

   public void initialize()
   {
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

      quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
      quadrupedWaypointsPositionTrajectoryGenerator.getPosition(desiredFootPosition);
      desiredFootVelocity.setToZero();

      desiredFootPosition.changeFrame(worldFrame);
      desiredFootVelocity.changeFrame(worldFrame);
      feedbackControlCommand.set(desiredFootPosition, desiredFootVelocity);
      feedbackControlCommand.setGains(parameters.getSolePositionGains());
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoints(trajectoryPointList);
      if (trajectoryPointList.getNumberOfTrajectoryPoints() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > trajectoryPointList.getTrajectoryTime())
         return virtualForceCommand;
      else
         return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > trajectoryPointList.getTrajectoryTime())
         return null;
      else
         return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return feedbackControlCommand;
   }
}
