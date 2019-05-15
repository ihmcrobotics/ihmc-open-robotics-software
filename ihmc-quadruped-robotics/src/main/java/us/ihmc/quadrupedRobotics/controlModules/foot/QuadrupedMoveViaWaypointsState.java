package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3DReadOnly zeroVector3D = new FrameVector3D(worldFrame);

   // Yo variables
   private final YoDouble robotTime;

   private final MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   private final FrameEuclideanTrajectoryPointList trajectoryPointList = new FrameEuclideanTrajectoryPointList();

   private final QuadrupedFootControlModuleParameters parameters;

   private final QuadrupedControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootVelocity = new FrameVector3D();
   private final FrameVector3D desiredFootAcceleration = new FrameVector3D();

   private WholeBodyControllerCoreMode controllerCoreMode = WholeBodyControllerCoreMode.VIRTUAL_MODEL;
   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private final FootSwitchInterface footSwitch;

   private final YoBoolean hasMinimumTimePassed;
   private final GlitchFilteredYoBoolean touchdownTrigger;

   private final YoDouble minimumTimeInState;
   private final YoDouble taskStartTime;
   private final YoDouble currentTrajectoryTime;


   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      ReferenceFrame soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      this.parameters = controllerToolbox.getFootControlModuleParameters();
      robotTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      footSwitch = controllerToolbox.getRuntimeEnvironment().getFootSwitches().get(robotQuadrant);

      // Create waypoint trajectory
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       worldFrame, registry);

      String namePrefix = robotQuadrant.getPascalCaseName();

      taskStartTime = new YoDouble(namePrefix + "MoveViaWaypointsTaskStartTime", registry);
      currentTrajectoryTime = new YoDouble(namePrefix + "MoveViaWaypointsCurrentTrajectoryTime", registry);
      hasMinimumTimePassed = new YoBoolean(namePrefix + "MoveViaWaypointsHasMinimumTimePassed", registry);

      minimumTimeInState = new YoDouble(namePrefix + "MinimumTimeInMoveViaWaypointsState", registry);
      minimumTimeInState.set(0.25);

      this.touchdownTrigger = new GlitchFilteredYoBoolean(namePrefix + "MoveViaWaypointsTouchdownTriggered", registry,
                                                          QuadrupedFootControlModuleParameters.getDefaultTouchdownTriggerWindow());

      RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      this.controllerCoreMode = controllerCoreMode;
      feedbackControlCommand.setControlMode(controllerCoreMode);
   }

   public void handleWaypointList(FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.set(trajectoryPointList);
      createSoleWaypointTrajectory();
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.getFootContactState(robotQuadrant).clear();
      footSwitch.reset();

      touchdownTrigger.set(false);
      hasMinimumTimePassed.set(false);

      desiredFootPosition.setToZero(controllerToolbox.getSoleReferenceFrame(robotQuadrant));
      desiredFootPosition.changeFrame(worldFrame);

      desiredFootVelocity.setToZero(worldFrame);

      createSoleWaypointTrajectory();
   }

   @Override
   public void doAction(double timeInState)
   {
      currentTrajectoryTime.set(robotTime.getDoubleValue() - taskStartTime.getValue());

      quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime.getDoubleValue());
      quadrupedWaypointsPositionTrajectoryGenerator.getPosition(desiredFootPosition);
      quadrupedWaypointsPositionTrajectoryGenerator.getVelocity(desiredFootVelocity);
      quadrupedWaypointsPositionTrajectoryGenerator.getAcceleration(desiredFootAcceleration);

      desiredFootPosition.changeFrame(worldFrame);
      desiredFootVelocity.changeFrame(worldFrame);
      desiredFootAcceleration.changeFrame(worldFrame);

      if (controllerCoreMode == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         feedbackControlCommand.setInverseDynamics(desiredFootPosition, desiredFootVelocity, desiredFootAcceleration);
      else if (controllerCoreMode == WholeBodyControllerCoreMode.VIRTUAL_MODEL)
         feedbackControlCommand.setVirtualModelControl(desiredFootPosition, desiredFootVelocity, zeroVector3D);
      else
         throw new UnsupportedOperationException("Unsupported control mode: " + controllerCoreMode);
      feedbackControlCommand.setGains(parameters.getSolePositionGains());
      feedbackControlCommand.setWeightsForSolver(parameters.getSolePositionWeights());

      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));
      if (hasMinimumTimePassed.getBooleanValue())
      {
         touchdownTrigger.update(footSwitch.hasFootHitGround());
      }
   }

   private boolean hasMinimumTimePassed(double timeInState)
   {
      return timeInState > minimumTimeInState.getValue();
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      QuadrupedFootControlModule.FootEvent eventToReturn = null;
      if (touchdownTrigger.getBooleanValue())
         eventToReturn = QuadrupedFootControlModule.FootEvent.LOADED;

      if (eventToReturn != null && stepTransitionCallback != null)
         stepTransitionCallback.onTouchDown(robotQuadrant);

      return eventToReturn;
   }

   @Override
   public void onExit()
   {
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.changeFrame(trajectoryPointList.getReferenceFrame());

      taskStartTime.set(robotTime.getDoubleValue());

      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoint(0.0, desiredFootPosition, desiredFootVelocity);
      quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoints(trajectoryPointList);
      if (trajectoryPointList.getNumberOfTrajectoryPoints() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return feedbackControlCommand;
   }
}
