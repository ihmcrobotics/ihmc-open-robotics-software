package us.ihmc.quadrupedRobotics.controller.force.states;

import controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedSteppingStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSpaceManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepCommandConsumer;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedSteppingState implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final QuadrupedStepCommandConsumer commandConsumer;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;

   private final YoEnum<QuadrupedSteppingRequestedEvent> lastEvent = new YoEnum<>("lastSteppingEvent", registry, QuadrupedSteppingRequestedEvent.class);
   private final StateMachine<QuadrupedSteppingStateEnum, QuadrupedController> stateMachine;
   private EventTrigger trigger;

   private final AtomicReference<QuadrupedSteppingRequestedEvent> requestedEvent = new AtomicReference<>();

   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedJointSpaceManager jointSpaceManager;

   private ControllerCoreOutputReadOnly controllerCoreOutput;

   private final QuadrupedSteppingStateChangeMessage quadrupedSteppingStateChangeMessage = new QuadrupedSteppingStateChangeMessage();

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
   private final WholeBodyControllerCore controllerCore;

   public QuadrupedSteppingState(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
                                 CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                 QuadrupedControlManagerFactory controlManagerFactory, YoVariableRegistry parentRegistry)
   {
      this.runtimeEnvironment = runtimeEnvironment;
      this.controllerToolbox = controllerToolbox;
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controlManagerFactory = controlManagerFactory;

      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      FullQuadrupedRobotModel fullRobotModel = runtimeEnvironment.getFullRobotModel();
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(runtimeEnvironment.getControlDT(), runtimeEnvironment.getGravity(),
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       fullRobotModel.getControllableOneDoFJoints(),
                                                                                       controllerToolbox.getReferenceFrames().getCenterOfMassFrame(),
                                                                                       runtimeEnvironment.getControllerCoreOptimizationSettings(),
                                                                                       runtimeEnvironment.getGraphicsListRegistry(), registry);
      controlCoreToolbox.setupForVirtualModelControlSolver(fullRobotModel.getBody(), controllerToolbox.getContactablePlaneBodies());
      FeedbackControlCommandList feedbackTemplate = controlManagerFactory.createFeedbackControlTemplate();
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackTemplate, runtimeEnvironment.getJointDesiredOutputList(), registry);
      controllerCoreOutput = controllerCore.getControllerCoreOutput();

      // Initialize input providers.
      stepMessageHandler = new QuadrupedStepMessageHandler(runtimeEnvironment.getRobotTimestamp(), controllerToolbox.getReferenceFrames(), registry);
      commandConsumer = new QuadrupedStepCommandConsumer(commandInputManager, stepMessageHandler, controllerToolbox, controlManagerFactory);

      this.stateMachine = buildStateMachine();

      parentRegistry.addChild(registry);
   }

   private StateMachine<QuadrupedSteppingStateEnum, QuadrupedController> buildStateMachine()
   {
      // Initialize controllers.
      final QuadrupedController standController = new QuadrupedStandController(controllerToolbox, controlManagerFactory, registry);
      final QuadrupedStepController stepController = new QuadrupedStepController(controllerToolbox, controlManagerFactory, stepMessageHandler, registry);
      final QuadrupedController soleWaypointController = new QuadrupedForceBasedSoleWaypointController(controllerToolbox, controlManagerFactory,
                                                                                                       stepMessageHandler, registry);

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
            quadrupedSteppingStateChangeMessage.setInitialSteppingControllerName(fromByte);
            quadrupedSteppingStateChangeMessage.setEndSteppingControllerName(toByte);
            statusMessageOutputManager.reportStatusMessage(quadrupedSteppingStateChangeMessage);
         }
      });

      trigger = factory.buildEventTrigger();

      return factory.build(QuadrupedSteppingStateEnum.STAND);
   }

   @Override
   public void onEntry()
   {
      commandInputManager.flushAllCommands();

      stepMessageHandler.clearFootTrajectory();
      stepMessageHandler.clearSteps();

      stateMachine.resetToInitialState();
   }

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();

   @Override
   public void doAction(double timeInState)
   {
      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
      balanceManager.computeAchievedCMP(achievedLinearMomentumRate);

      commandConsumer.update();
      commandConsumer.consumeFootCommands();

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

      submitControllerCoreCommands();

      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();
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

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotQuadrant);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         //planeContactStateCommand.setUseHighCoPDamping(false);
         controllerCoreCommand.addVirtualModelControlCommand(planeContactStateCommand);
      }

      controllerCoreCommand.addFeedbackControlCommand(bodyOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(bodyOrientationManager.getVirtualModelControlCommand());

      controllerCoreCommand.addVirtualModelControlCommand(balanceManager.getVirtualModelControlCommand());

      controllerCoreCommand.addFeedbackControlCommand(jointSpaceManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(jointSpaceManager.getVirtualModelControlCommand());
   }

   @Override
   public void onExit()
   {

   }
}
