package us.ihmc.quadrupedRobotics.controller.states;

import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedSteppingStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSettingsManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.*;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepCommandConsumer;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointControlParameters;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedSteppingState implements QuadrupedController, QuadrupedStepTransitionCallback
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final QuadrupedStepCommandConsumer commandConsumer;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;

   private final YoEnum<QuadrupedSteppingRequestedEvent> lastEvent = new YoEnum<>("lastSteppingEvent", registry, QuadrupedSteppingRequestedEvent.class);
   private final StateMachine<QuadrupedSteppingStateEnum, QuadrupedController> stateMachine;
   private EventTrigger trigger;

   private final GroundPlaneEstimator groundPlaneEstimator;
   private final GroundPlaneEstimator upcomingGroundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint3D> groundPlanePositions;
   private final QuadrantDependentList<YoFramePoint3D> upcomingGroundPlanePositions;

   private final YoBoolean onLiftOffTriggered = new YoBoolean("onLiftOffTriggered", registry);
   private final YoBoolean onTouchDownTriggered = new YoBoolean("onTouchDownTriggered", registry);

   private final AtomicReference<QuadrupedSteppingRequestedEvent> requestedEvent = new AtomicReference<>();

   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedJointSettingsManager jointSpaceManager;

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final QuadrupedSteppingStateChangeMessage quadrupedSteppingStateChangeMessage = new QuadrupedSteppingStateChangeMessage();

   private final QuadrantDependentList<QuadrupedFootstepStatusMessage> footstepStatusMessages = new QuadrantDependentList<>();
   private final YoInteger stepIndex = new YoInteger("currentStepIndex", registry);
   private final QuadrupedGroundPlaneMessage groundPlaneMessage = new QuadrupedGroundPlaneMessage();

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);
   private final ControllerCoreCommand controllerCoreCommand;
   private final WholeBodyControllerCore controllerCore;
   private final JointDesiredOutputList jointDesiredOutputList;


   public QuadrupedSteppingState(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerToolbox controllerToolbox,
                                 CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                 QuadrupedControlManagerFactory controlManagerFactory, YoVariableRegistry parentRegistry)
   {
      this.runtimeEnvironment = runtimeEnvironment;
      this.controllerToolbox = controllerToolbox;
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controlManagerFactory = controlManagerFactory;
      this.jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();

      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      controllerCoreCommand = new ControllerCoreCommand(controllerToolbox.getControllerCoreMode());

      FullQuadrupedRobotModel fullRobotModel = runtimeEnvironment.getFullRobotModel();
      // FIXME this will be wrong if we add in neck joints
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(runtimeEnvironment.getControlDT(), runtimeEnvironment.getGravity(),
                                                                                       fullRobotModel.getRootJoint(), allJoints,
                                                                                       controllerToolbox.getReferenceFrames().getCenterOfMassFrame(),
                                                                                       runtimeEnvironment.getControllerCoreOptimizationSettings(),
                                                                                       runtimeEnvironment.getGraphicsListRegistry(), registry);
      controlCoreToolbox.setupForVirtualModelControlSolver(fullRobotModel.getBody(), controllerToolbox.getContactablePlaneBodies());
      controlCoreToolbox.setupForInverseKinematicsSolver();

      FeedbackControlCommandList feedbackTemplate = controlManagerFactory.createFeedbackControlTemplate();
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackTemplate, jointDesiredOutputList, registry);
      controllerCoreOutput = controllerCore.getControllerCoreOutput();

      // Initialize input providers.
      stepMessageHandler = new QuadrupedStepMessageHandler(runtimeEnvironment.getRobotTimestamp(), registry);
      commandConsumer = new QuadrupedStepCommandConsumer(commandInputManager, stepMessageHandler, controllerToolbox, controlManagerFactory);

      // step planner
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      upcomingGroundPlaneEstimator = controllerToolbox.getUpcomingGroundPlaneEstimator();
      groundPlanePositions = controllerToolbox.getGroundPlanePositions();
      upcomingGroundPlanePositions = controllerToolbox.getUpcomingGroundPlanePositions();

      this.stateMachine = buildStateMachine();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footstepStatusMessages.set(robotQuadrant, new QuadrupedFootstepStatusMessage());
      }

      parentRegistry.addChild(registry);
   }

   private StateMachine<QuadrupedSteppingStateEnum, QuadrupedController> buildStateMachine()
   {
      // Initialize controllers.
      final QuadrupedController standController = new QuadrupedStandController(controllerToolbox, controlManagerFactory, registry);
      final QuadrupedStepController stepController = new QuadrupedStepController(controllerToolbox, controlManagerFactory, stepMessageHandler, registry);
      final QuadrupedController soleWaypointController = new QuadrupedSoleWaypointController(controllerToolbox, controlManagerFactory, stepMessageHandler,
                                                                                             registry);

      EventBasedStateMachineFactory<QuadrupedSteppingStateEnum, QuadrupedController> factory = new EventBasedStateMachineFactory<>(
            QuadrupedSteppingStateEnum.class);
      factory.setNamePrefix("stepping").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());
      factory.buildYoEventTrigger("stepTrigger", QuadrupedSteppingRequestedEvent.class);

      factory.addState(QuadrupedSteppingStateEnum.STAND, standController);
      factory.addState(QuadrupedSteppingStateEnum.STEP, stepController);
      factory.addState(QuadrupedSteppingStateEnum.SOLE_WAYPOINT, soleWaypointController);

      // Add automatic transitions that lead into the stand state.
      factory.addTransition(ControllerEvent.DONE, QuadrupedSteppingStateEnum.STEP, QuadrupedSteppingStateEnum.STAND);

      // Sole Waypoint events
      factory.addTransition(QuadrupedSteppingRequestedEvent.REQUEST_SOLE_WAYPOINT, QuadrupedSteppingStateEnum.STAND, QuadrupedSteppingStateEnum.SOLE_WAYPOINT);
      factory.addTransition(ControllerEvent.DONE, QuadrupedSteppingStateEnum.SOLE_WAYPOINT, QuadrupedSteppingStateEnum.STAND);
      factory.addTransition(ControllerEvent.FAIL, QuadrupedSteppingStateEnum.SOLE_WAYPOINT, QuadrupedSteppingStateEnum.STAND);

      // Manually triggered events to transition to main controllers.
      factory.addTransition(QuadrupedSteppingRequestedEvent.REQUEST_STEP, QuadrupedSteppingStateEnum.STAND, QuadrupedSteppingStateEnum.STEP);

      Runnable stepToStandCallback = () -> stepController.halt();
      factory.addCallback(QuadrupedSteppingRequestedEvent.REQUEST_STAND, QuadrupedSteppingStateEnum.STEP, stepToStandCallback);

      factory.addStateChangedListener(new StateChangedListener<QuadrupedSteppingStateEnum>()
      {
         @Override
         public void stateChanged(QuadrupedSteppingStateEnum from, QuadrupedSteppingStateEnum to)
         {
            byte fromByte = from == null ? -1 : from.toByte();
            byte toByte = to == null ? -1 : to.toByte();
            quadrupedSteppingStateChangeMessage.setInitialQuadrupedSteppingStateEnum(fromByte);
            quadrupedSteppingStateChangeMessage.setEndQuadrupedSteppingStateEnum(toByte);
            statusMessageOutputManager.reportStatusMessage(quadrupedSteppingStateChangeMessage);
         }
      });

      trigger = factory.buildEventTrigger();

      return factory.build(QuadrupedSteppingStateEnum.STAND);
   }

   @Override
   public void onLiftOff(QuadrupedTimedStep step)
   {
      RobotQuadrant quadrant = step.getRobotQuadrant();

      // update ground plane estimate
      tempPoint.setToZero(controllerToolbox.getSoleReferenceFrame(quadrant));
      groundPlanePositions.get(quadrant).setMatchingFrame(tempPoint);

      step.getGoalPosition(tempPoint);
      tempPoint.changeFrame(worldFrame);
      upcomingGroundPlanePositions.get(quadrant).setMatchingFrame(tempPoint);

      onLiftOffTriggered.set(true);

      // report footstep status message
      stepIndex.increment();
      double currentTime = runtimeEnvironment.getRobotTimestamp().getDoubleValue();
      QuadrupedFootstepStatusMessage footstepStatusMessage = footstepStatusMessages.get(quadrant);

      footstepStatusMessage.setFootstepQuadrant(quadrant.toByte());
      footstepStatusMessage.setFootstepStatus(QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED);
      footstepStatusMessage.setSequenceId(stepIndex.getIntegerValue());
      footstepStatusMessage.getDesiredStepInterval().setStartTime(step.getTimeInterval().getStartTime());
      footstepStatusMessage.getDesiredStepInterval().setEndTime(step.getTimeInterval().getEndTime());
      footstepStatusMessage.getActualStepInterval().setStartTime(currentTime);
      footstepStatusMessage.getActualStepInterval().setEndTime(Double.NaN);
      step.getGoalPosition(footstepStatusMessage.getDesiredTouchdownPositionInWorld());
      statusMessageOutputManager.reportStatusMessage(footstepStatusMessage);
   }

   @Override
   public void onTouchDown(RobotQuadrant thisStepQuadrant)
   {
      onTouchDownTriggered.set(true);

      // report footstep status message
      QuadrupedFootstepStatusMessage footstepStatusMessage = footstepStatusMessages.get(thisStepQuadrant);
      tempPoint.setToZero(controllerToolbox.getSoleReferenceFrame(thisStepQuadrant));

      tempPoint.changeFrame(worldFrame);

      double currentTime = runtimeEnvironment.getRobotTimestamp().getDoubleValue();

      footstepStatusMessage.setFootstepQuadrant(thisStepQuadrant.toByte());
      footstepStatusMessage.setFootstepStatus(QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED);
      footstepStatusMessage.getActualStepInterval().setEndTime(currentTime);
      footstepStatusMessage.getActualTouchdownPositionInWorld().set(tempPoint);
      footstepStatusMessage.getDesiredTouchdownPositionInWorld().add(balanceManager.getStepAdjustment(thisStepQuadrant));
      statusMessageOutputManager.reportStatusMessage(footstepStatusMessage);

      stepMessageHandler.shiftPlanBasedOnStepAdjustment(balanceManager.getStepAdjustment(thisStepQuadrant));

      balanceManager.completedStep(thisStepQuadrant);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.update();

      commandInputManager.clearAllCommands();

      onLiftOffTriggered.set(false);
      onTouchDownTriggered.set(false);

      stepMessageHandler.clearFootTrajectory();
      stepMessageHandler.clearSteps();

      // initialize ground plane
      groundPlaneEstimator.clearContactPoints();
      upcomingGroundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerToolbox.getContactStates().put(robotQuadrant, ContactState.IN_CONTACT);

         tempPoint.setToZero(controllerToolbox.getSoleReferenceFrame(robotQuadrant));
         groundPlanePositions.get(robotQuadrant).setMatchingFrame(tempPoint);
         upcomingGroundPlanePositions.get(robotQuadrant).setMatchingFrame(tempPoint);

         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
         upcomingGroundPlaneEstimator.addContactPoint(upcomingGroundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();
      upcomingGroundPlaneEstimator.compute();

      controllerCore.initialize();
      feetManager.registerStepTransitionCallback(this);

      stateMachine.resetToInitialState();
   }

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();

   @Override
   public void doAction(double timeInState)
   {
      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
      balanceManager.computeAchievedCMP(achievedLinearMomentumRate);

      controllerToolbox.update();

      if (commandInputManager.isNewCommandAvailable(QuadrupedRequestedSteppingStateCommand.class))
      {
         requestedEvent.set(commandInputManager.pollNewestCommand(QuadrupedRequestedSteppingStateCommand.class).getRequestedSteppingState());
      }

      QuadrupedSteppingRequestedEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         lastEvent.set(reqEvent);
         trigger.fireEvent(reqEvent);
      }

      commandConsumer.update();
      commandConsumer.consumeFootCommands();
      commandConsumer.consumeBodyCommands();
      updateAndReportGroundPlaneEstimate();

      if (stepMessageHandler.isStepPlanAvailable())
      {
         if (stateMachine.getCurrentStateKey() == QuadrupedSteppingStateEnum.STAND)
         {
            // trigger step event steps are available in stand state
            lastEvent.set(QuadrupedSteppingRequestedEvent.REQUEST_STEP);
            trigger.fireEvent(QuadrupedSteppingRequestedEvent.REQUEST_STEP);
         }
      }
      if (stepMessageHandler.hasFootTrajectoryForSolePositionControl())
      {
         if (stateMachine.getCurrentStateKey() == QuadrupedSteppingStateEnum.STAND)
         {
            // trigger step event if sole waypoints are available in stand state
            lastEvent.set(QuadrupedSteppingRequestedEvent.REQUEST_SOLE_WAYPOINT);
            trigger.fireEvent(QuadrupedSteppingRequestedEvent.REQUEST_SOLE_WAYPOINT);
         }
      }

      // update controller state machine
      stateMachine.doActionAndTransition();

      jointSpaceManager.compute();

      handleChangeInContactState();

      submitControllerCoreCommands();

      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();

      setLowLevelJointControlParameters();
   }

   private void setLowLevelJointControlParameters()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedJointControlParameters jointControlParameters = controllerToolbox.getJointControlParameters();
         PDGainsReadOnly gains;
         if (controllerToolbox.getFootContactState(robotQuadrant).inContact())
            gains = jointControlParameters.getSteppingLoadedJointGains();
         else
            gains = jointControlParameters.getSteppingUnloadedJointGains();


         FullQuadrupedRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
         List<OneDoFJoint> legJoints = fullRobotModel.getLegJointsList(robotQuadrant);
         for (int i = 0; i < legJoints.size(); i++)
         {
            OneDoFJoint legJoint = legJoints.get(i);
            JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(legJoint);
            jointDesiredOutput.setStiffness(gains.getKp());
            jointDesiredOutput.setDamping(gains.getKd());
            jointDesiredOutput.setMaxPositionError(gains.getMaximumFeedback());
            jointDesiredOutput.setMaxVelocityError(gains.getMaximumFeedbackRate());

            jointDesiredOutput.setControlMode(jointControlParameters.getSteppingJointMode());
         }
      }
   }

   private void updateAndReportGroundPlaneEstimate()
   {
      // update ground plane estimate
      groundPlaneEstimator.clearContactPoints();
      upcomingGroundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
         upcomingGroundPlaneEstimator.addContactPoint(upcomingGroundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();
      upcomingGroundPlaneEstimator.compute();

      groundPlaneEstimator.getPlanePoint(tempPoint);
      groundPlaneEstimator.getPlaneNormal(tempVector);
      groundPlaneMessage.region_origin_.set(tempPoint);
      groundPlaneMessage.region_normal_.set(tempVector);
      statusMessageOutputManager.reportStatusMessage(groundPlaneMessage);
   }

   private void handleChangeInContactState()
   {
      // update accumulated step adjustment
      if (onLiftOffTriggered.getBooleanValue())
      {
         onLiftOffTriggered.set(false);
      }
      if (onTouchDownTriggered.getBooleanValue())
      {
         onTouchDownTriggered.set(false);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerToolbox.getContactStates().put(robotQuadrant, feetManager.getContactState(robotQuadrant));
      }
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return null;
   }

   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);

   private void submitControllerCoreCommands()
   {
      planeContactStateCommandPool.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotQuadrant));
         controllerCoreCommand.addVirtualModelControlCommand(feetManager.getVirtualModelControlCommand(robotQuadrant));
         controllerCoreCommand.addInverseKinematicsCommand(feetManager.getInverseKinematicsCommand(robotQuadrant));

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotQuadrant);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         controllerCoreCommand.addVirtualModelControlCommand(planeContactStateCommand);
      }

      controllerCoreCommand.addFeedbackControlCommand(bodyOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(bodyOrientationManager.getVirtualModelControlCommand());
      controllerCoreCommand.addInverseKinematicsCommand(bodyOrientationManager.getInverseKinematicsCommand());

      controllerCoreCommand.addVirtualModelControlCommand(balanceManager.getVirtualModelControlCommand());
      controllerCoreCommand.addInverseKinematicsCommand(balanceManager.getInverseKinematicsCommand());

      controllerCoreCommand.addFeedbackControlCommand(jointSpaceManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(jointSpaceManager.getVirtualModelControlCommand());
      controllerCoreCommand.addInverseKinematicsCommand(jointSpaceManager.getInverseKinematicsCommand());
   }

   @Override
   public void onExit()
   {

   }
}
