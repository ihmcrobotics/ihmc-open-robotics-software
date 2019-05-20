package us.ihmc.quadrupedRobotics.controller.states;

import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedRequestedSteppingStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSpaceManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepCommandConsumer;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.GroundPlaneEstimator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventState;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedWalkingControllerState extends HighLevelControllerState implements QuadrupedStepTransitionCallback
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final QuadrupedStepCommandConsumer commandConsumer;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;

   private final YoEnum<QuadrupedSteppingRequestedEvent> lastEvent = new YoEnum<>("lastSteppingEvent", registry, QuadrupedSteppingRequestedEvent.class);
   private final StateMachine<QuadrupedSteppingStateEnum, EventState> stateMachine;
   private EventTrigger trigger;

   private final GroundPlaneEstimator groundPlaneEstimator;
   private final GroundPlaneEstimator upcomingGroundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint3D> groundPlanePositions;
   private final QuadrantDependentList<YoFramePoint3D> upcomingGroundPlanePositions;

   private final QuadrantDependentList<Set<String>> legJointNames = new QuadrantDependentList<>();

   private final YoBoolean onLiftOffTriggered = new YoBoolean("onLiftOffTriggered", registry);
   private final YoBoolean onTouchDownTriggered = new YoBoolean("onTouchDownTriggered", registry);

   private final AtomicReference<QuadrupedSteppingRequestedEvent> requestedEvent = new AtomicReference<>();

   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedJointSpaceManager jointSpaceManager;

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final QuadrupedSteppingStateChangeMessage quadrupedSteppingStateChangeMessage = new QuadrupedSteppingStateChangeMessage();

   private final QuadrantDependentList<QuadrupedFootstepStatusMessage> footstepStatusMessages = new QuadrantDependentList<>();
   private final YoInteger stepIndex = new YoInteger("currentStepIndex", registry);
   private final QuadrupedGroundPlaneMessage groundPlaneMessage = new QuadrupedGroundPlaneMessage();

   private final boolean deactivateAccelerationIntegrationInWBC;

   private boolean requestIntegratorReset = false;
   private final YoBoolean yoRequestingIntegratorReset = new YoBoolean("RequestingIntegratorReset", registry);
   private final DoubleParameter loadPercentageForGroundPlane = new DoubleParameter("loadPercentageForGroundPlaneUpdate", registry, 0.25);

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);
   private final EnumProvider<WholeBodyControllerCoreMode> controllerCoreMode;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private final WholeBodyControllerCore controllerCore;

   public QuadrupedWalkingControllerState(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerToolbox controllerToolbox,
                                          CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                          QuadrupedControlManagerFactory controlManagerFactory)
   {
      super(HighLevelControllerName.WALKING, runtimeEnvironment.getHighLevelControllerParameters(), controllerToolbox.getFullRobotModel().getControllableOneDoFJoints());
      this.runtimeEnvironment = runtimeEnvironment;
      this.controllerToolbox = controllerToolbox;
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controlManagerFactory = controlManagerFactory;

      controllerCoreMode = new EnumParameter<>("controllerCoreMode", registry, WholeBodyControllerCoreMode.class, false, WholeBodyControllerCoreMode.VIRTUAL_MODEL);

      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      FullQuadrupedRobotModel fullRobotModel = runtimeEnvironment.getFullRobotModel();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotQuadrant);
         OneDoFJointBasics[] legJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(fullRobotModel.getBody(), foot), OneDoFJointBasics.class);
         Set<String> jointNames = new HashSet<>();
         Arrays.stream(legJoints).forEach(legJoint -> jointNames.add(legJoint.getName()));
         legJointNames.put(robotQuadrant, jointNames);
      }


      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(runtimeEnvironment.getControlDT(), runtimeEnvironment.getGravity(),
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       fullRobotModel.getRootJoint().subtreeArray(),
                                                                                       controllerToolbox.getReferenceFrames().getCenterOfMassFrame(),
                                                                                       runtimeEnvironment.getControllerCoreOptimizationSettings(),
                                                                                       runtimeEnvironment.getGraphicsListRegistry(), registry);
      controlCoreToolbox.setupForVirtualModelControlSolver(fullRobotModel.getBody(), controllerToolbox.getContactablePlaneBodies());
      controlCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());
      controlCoreToolbox.setJointPrivilegedConfigurationParameters(runtimeEnvironment.getPrivilegedConfigurationParameters());
      FeedbackControlCommandList feedbackTemplate = controlManagerFactory.createFeedbackControlTemplate();
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackTemplate, runtimeEnvironment.getJointDesiredOutputList(), registry);
      controllerCoreOutput = controllerCore.getControllerCoreOutput();

      deactivateAccelerationIntegrationInWBC = runtimeEnvironment.getHighLevelControllerParameters().deactivateAccelerationIntegrationInTheWBC();

      // Initialize input providers.
      stepMessageHandler = new QuadrupedStepMessageHandler(runtimeEnvironment.getRobotTimestamp(), runtimeEnvironment.getControlDT(), registry);
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
   }

   private StateMachine<QuadrupedSteppingStateEnum, EventState> buildStateMachine()
   {
      // Initialize controllers.
      QuadrupedStandController standController = new QuadrupedStandController(controllerToolbox, controlManagerFactory, registry);
      QuadrupedStepController stepController = new QuadrupedStepController(controllerToolbox, controlManagerFactory, stepMessageHandler, registry);
      QuadrupedSoleWaypointController soleWaypointController = new QuadrupedSoleWaypointController(controllerToolbox, controlManagerFactory, stepMessageHandler,
                                                                                                   registry);

      EventBasedStateMachineFactory<QuadrupedSteppingStateEnum, EventState> factory = new EventBasedStateMachineFactory<>(QuadrupedSteppingStateEnum.class);
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

      Runnable stepToStandCallback = stepController::halt;
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

      tempPoint.setIncludingFrame(step.getReferenceFrame(), step.getGoalPosition());
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
      footstepStatusMessage.getDesiredTouchdownPositionInWorld().set(step.getGoalPosition());
      statusMessageOutputManager.reportStatusMessage(footstepStatusMessage);

      balanceManager.beganStep();

      controllerToolbox.getFallDetector().setNextFootstep(quadrant, step);
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

      stepMessageHandler.onTouchDown(thisStepQuadrant);
      stepMessageHandler.shiftPlanBasedOnStepAdjustment(balanceManager.getStepAdjustment(thisStepQuadrant));

      balanceManager.completedStep(thisStepQuadrant);

      controllerToolbox.getFallDetector().setNextFootstep(thisStepQuadrant, null);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.update();

      commandInputManager.clearAllCommands();

      onLiftOffTriggered.set(false);
      onTouchDownTriggered.set(false);

      JointDesiredOutputList jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();
      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
         jointDesiredOutputList.getJointDesiredOutput(i).setControlMode(JointDesiredControlMode.EFFORT);

      stepMessageHandler.clearFootTrajectory();
      stepMessageHandler.clearSteps();

      // initialize ground plane
      groundPlaneEstimator.clearContactPoints();
      upcomingGroundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         tempPoint.setToZero(controllerToolbox.getSoleReferenceFrame(robotQuadrant));
         tempPoint.changeFrame(worldFrame);
         groundPlanePositions.get(robotQuadrant).set(tempPoint);
         upcomingGroundPlanePositions.get(robotQuadrant).set(tempPoint);

         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
         upcomingGroundPlaneEstimator.addContactPoint(upcomingGroundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();
      upcomingGroundPlaneEstimator.compute();

      feetManager.registerStepTransitionCallback(this);

      balanceManager.initialize();

      stateMachine.resetToInitialState();

      initialize();
   }

   public void initialize()
   {
      controllerCore.initialize();
      feetManager.setControllerCoreMode(controllerCoreMode.getValue());
      bodyOrientationManager.setControllerCoreMode(controllerCoreMode.getValue());
      requestIntegratorReset = true;
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
      stateMachine.doTransitions();
      stateMachine.doAction();
//      stateMachine.doActionAndTransition();

      jointSpaceManager.compute();

      handleChangeInContactState();

      controllerCoreCommand.setControllerCoreMode(controllerCoreMode.getValue());

      submitControllerCoreCommands();

      JointDesiredOutputList stateSpecificJointSettings = getStateSpecificJointSettings();

      if (requestIntegratorReset)
      {
         stateSpecificJointSettings.requestIntegratorReset();
         requestIntegratorReset = false;
         yoRequestingIntegratorReset.set(true);
      }
      else
      {
         yoRequestingIntegratorReset.set(false);
      }

      JointAccelerationIntegrationCommand accelerationIntegrationCommand = getAccelerationIntegrationCommand();
      if (!deactivateAccelerationIntegrationInWBC)
      {
         controllerCoreCommand.addVirtualModelControlCommand(accelerationIntegrationCommand);
         controllerCoreCommand.addInverseDynamicsCommand(accelerationIntegrationCommand);
      }
      controllerCoreCommand.completeLowLevelJointData(stateSpecificJointSettings);

      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();
   }

   private void updateAndReportGroundPlaneEstimate()
   {
      // update ground plane estimate
      groundPlaneEstimator.clearContactPoints();
      upcomingGroundPlaneEstimator.clearContactPoints();
      QuadrantDependentList<FootSwitchInterface> footSwitches = controllerToolbox.getRuntimeEnvironment().getFootSwitches();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (footSwitches.get(robotQuadrant).computeFootLoadPercentage() > loadPercentageForGroundPlane.getValue())
         {
            groundPlanePositions.get(robotQuadrant).setFromReferenceFrame(controllerToolbox.getSoleReferenceFrame(robotQuadrant));
            upcomingGroundPlanePositions.get(robotQuadrant).setFromReferenceFrame(controllerToolbox.getSoleReferenceFrame(robotQuadrant));
         }

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

   // FIXME does this do anything anymore?
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
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return false;
   }

   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);

   private void submitControllerCoreCommands()
   {
      planeContactStateCommandPool.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotQuadrant));
         controllerCoreCommand.addVirtualModelControlCommand(feetManager.getVirtualModelControlCommand(robotQuadrant));
         controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(robotQuadrant));

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotQuadrant);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         controllerCoreCommand.addVirtualModelControlCommand(planeContactStateCommand);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }

      controllerCoreCommand.addFeedbackControlCommand(bodyOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(bodyOrientationManager.getVirtualModelControlCommand());
      controllerCoreCommand.addInverseDynamicsCommand(bodyOrientationManager.getInverseDynamicsCommand());

      controllerCoreCommand.addVirtualModelControlCommand(balanceManager.getVirtualModelControlCommand());
      controllerCoreCommand.addInverseDynamicsCommand(balanceManager.getInverseDynamicsCommand());

      controllerCoreCommand.addFeedbackControlCommand(jointSpaceManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(jointSpaceManager.getVirtualModelControlCommand());
      controllerCoreCommand.addInverseDynamicsCommand(jointSpaceManager.getInverseDynamicsCommand());
   }

   @Override
   public void onExit()
   {

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public boolean isJointLoadBearing(String jointName)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         boolean legLoaded = feetManager.getContactState(robotQuadrant).isLoadBearing();
         if (legLoaded && legJointNames.get(robotQuadrant).contains(jointName))
            return true;
      }

      return false;
   }
}
