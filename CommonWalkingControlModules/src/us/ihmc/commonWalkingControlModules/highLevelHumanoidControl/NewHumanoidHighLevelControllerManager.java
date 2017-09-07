package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NewHighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.NewHighLevelControllerStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.*;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.LowLevelJointDataReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

public class NewHumanoidHighLevelControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final GenericStateMachine<NewHighLevelControllerStates, NewHighLevelControllerState> stateMachine;
   private final NewHighLevelControllerStates initialControllerState;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoBoolean isListeningToHighLevelStateMessage = new YoBoolean("isListeningToHighLevelStateMessage", registry);
   private final YoEnum<NewHighLevelControllerStates> requestedHighLevelControllerState;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final ControllerCoreOutputReadOnly controllerCoreOutput;
   private final WholeBodyControllerCore controllerCore;

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ICPTrajectoryPlannerParameters icpPlannerParameters;

   private final EnumMap<NewHighLevelControllerStates, NewHighLevelControllerState> highLevelControllerStates = new EnumMap<>(NewHighLevelControllerStates.class);

   private final AtomicReference<NewHighLevelControllerStates> fallbackControllerForFailureReference = new AtomicReference<>();

   private final NewHighLevelControllerStateChangeStatusMessage highLevelStateChangeStatusMessage = new NewHighLevelControllerStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);

   public NewHumanoidHighLevelControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                                WholeBodyControllerCore controllerCore, NewHighLevelControllerStates initialControllerState,
                                                HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters,
                                                ICPTrajectoryPlannerParameters icpPlannerParameters, YoEnum<NewHighLevelControllerStates> requestedHighLevelControllerState,
                                                EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerStateFactories,
                                                ArrayList<ControllerStateTransitionFactory<NewHighLevelControllerStates>> controllerTransitionFactories,
                                                HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox, CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      YoDouble yoTime = controllerToolbox.getYoTime();
      this.controllerCoreOutput = controllerCoreOutput;
      this.controllerCore = controllerCore;
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.icpPlannerParameters = icpPlannerParameters;
      this.requestedHighLevelControllerState = requestedHighLevelControllerState;
      this.requestedHighLevelControllerState.set(initialControllerState);

      this.stateMachine = setUpStateMachine(controllerStateFactories, controllerTransitionFactories, managerFactory, yoTime, registry);

      isListeningToHighLevelStateMessage.set(true);

      for (NewHighLevelControllerState highLevelControllerState : highLevelControllerStates.values())
      {
         this.registry.addChild(highLevelControllerState.getYoVariableRegistry());
      }
      this.initialControllerState = initialControllerState;
      this.controllerToolbox = controllerToolbox;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.registry.addChild(controllerToolbox.getYoVariableRegistry());

      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controllerToolbox.getFullRobotModel().getOneDoFJoints(), registry);

      controllerToolbox.attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2D fallingDirection)
         {
            NewHighLevelControllerStates fallbackController = fallbackControllerForFailureReference.get();
            if (fallbackController != null)
               requestedHighLevelControllerState.set(fallbackController);
         }
      });
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.addChild(registryToAdd);
   }

   public void requestHighLevelControllerState(NewHighLevelControllerStates requestedHighLevelControllerState)
   {
      this.requestedHighLevelControllerState.set(requestedHighLevelControllerState);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      isListeningToHighLevelStateMessage.set(isListening);
   }

   @Override
   public void initialize()
   {
      controllerCore.initialize();
      controllerToolbox.initialize();
      stateMachine.setCurrentState(initialControllerState);
   }

   @Override
   public void doControl()
   {
      if (isListeningToHighLevelStateMessage.getBooleanValue())
      {
         if (commandInputManager.isNewCommandAvailable(NewHighLevelControllerStateCommand.class))
         {
            requestedHighLevelControllerState.set(commandInputManager.pollNewestCommand(NewHighLevelControllerStateCommand.class).getHighLevelControllerState());
         }
      }

      stateMachine.checkTransitionConditions();
      highLevelControllerTimer.startMeasurement();
      stateMachine.doAction();
      highLevelControllerTimer.stopMeasurement();

      reportDesiredCenterOfPressureForEstimator();
      copyJointDesiredsToJoints();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return this.getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   /**
    * Warmup the walking behavior by running all states for a number of iterations.
    *
    * Also warms up the controller core
    *
    * @param iterations number of times to run a single state
    * @param walkingState
    */
   public void warmup(int iterations, WalkingHighLevelHumanoidController walkingState)
   {
      PrintTools.info(this, "Starting JIT warmup routine");
      ArrayList<WalkingStateEnum> states = new ArrayList<>();
      controllerCore.initialize();
      walkingState.doTransitionIntoAction();

      walkingState.getOrderedWalkingStatesForWarmup(states);
      for(WalkingStateEnum state : states)
      {
         PrintTools.info(this, "Warming up " + state);
         for(int i = 0; i < iterations; i++)
         {
            walkingState.warmupStateIteration(state);
            ControllerCoreCommand controllerCoreCommandList = walkingState.getControllerCoreCommand();
            controllerCore.submitControllerCoreCommand(controllerCoreCommandList);
            controllerCore.compute();
         }
      }

      walkingState.doTransitionOutOfAction();
      walkingState.getControllerCoreCommand().clear();
      PrintTools.info(this, "Finished JIT warmup routine");
   }

   private GenericStateMachine<NewHighLevelControllerStates, NewHighLevelControllerState> setUpStateMachine(EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerStateFactories,
                                                                                                            ArrayList<ControllerStateTransitionFactory<NewHighLevelControllerStates>> controllerTransitionFactories,
                                                                                                            HighLevelControlManagerFactory managerFactory, YoDouble yoTime, YoVariableRegistry registry)
   {
      GenericStateMachine<NewHighLevelControllerStates, NewHighLevelControllerState> highLevelStateMachine = new GenericStateMachine<>("highLevelControllerState", "switchTimeName",
                                                                                                                                       NewHighLevelControllerStates.class, yoTime, registry);


      // create controller states
      for (HighLevelControllerStateFactory controllerStateFactory : controllerStateFactories.values())
      {
         // set the controller core output
         NewHighLevelControllerState highLevelControllerState = controllerStateFactory.getOrCreateControllerState(controllerStateFactories, controllerToolbox, highLevelControllerParameters,
                                                                                                                  commandInputManager, statusMessageOutputManager, managerFactory,
                                                                                                                  walkingControllerParameters, icpPlannerParameters, controllerCore);
         highLevelControllerState.setControllerCoreOutput(controllerCoreOutput);

         // add the controller to the state machine
         highLevelStateMachine.addState(highLevelControllerState);

         highLevelControllerStates.put(controllerStateFactory.getStateEnum(), highLevelControllerState);
      }

      // create controller transitions
      for (ControllerStateTransitionFactory<NewHighLevelControllerStates> controllerStateTransitionFactory : controllerTransitionFactories)
      {
         StateTransition<NewHighLevelControllerStates> stateTransition = controllerStateTransitionFactory.getOrCreateStateTransition(highLevelControllerStates);

         NewHighLevelControllerState state = highLevelControllerStates.get(controllerStateTransitionFactory.getStateToAttachEnum());
         state.addStateTransition(stateTransition);
      }

      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<NewHighLevelControllerStates>()
      {
         @Override
         public void stateChanged(State<NewHighLevelControllerStates> oldState, State<NewHighLevelControllerStates> newState, double time)
         {
            highLevelStateChangeStatusMessage.setStateChange(oldState.getStateEnum(), newState.getStateEnum());
            statusMessageOutputManager.reportStatusMessage(highLevelStateChangeStatusMessage);
         }
      });

      return highLevelStateMachine;
   }


   private final SideDependentList<FramePoint2D> desiredFootCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());

   private void reportDesiredCenterOfPressureForEstimator()
   {
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = controllerToolbox.getContactableFeet();
      FullHumanoidRobotModel fullHumanoidRobotModel = controllerToolbox.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.getDesiredCenterOfPressure(contactableFeet.get(robotSide), desiredFootCoPs.get(robotSide));
         centerOfPressureDataHolderForEstimator.setCenterOfPressure(desiredFootCoPs.get(robotSide), fullHumanoidRobotModel.getFoot(robotSide));
      }
   }

   private void copyJointDesiredsToJoints()
   {
      LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithLowLevelData(); jointIndex++)
      {
         OneDoFJoint controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         LowLevelJointDataReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");

         if (lowLevelJointData.hasDesiredPosition())
            controlledJoint.setqDesired(lowLevelJointData.getDesiredPosition());

         if (lowLevelJointData.hasDesiredVelocity())
            controlledJoint.setQdDesired(lowLevelJointData.getDesiredVelocity());

         if (lowLevelJointData.hasDesiredAcceleration())
            controlledJoint.setQddDesired(lowLevelJointData.getDesiredAcceleration());

         if (lowLevelJointData.hasDesiredTorque())
            controlledJoint.setTau(lowLevelJointData.getDesiredTorque());
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }


}
