package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewWalkingControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelStateCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NewHighLevelStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.NewHighLevelControllerStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.*;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
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

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final ControllerCoreOutputReadOnly controllerCoreOutput;
   private final WholeBodyControllerCore controllerCore;

   private final AtomicReference<NewHighLevelControllerStates> fallbackControllerForFailureReference = new AtomicReference<>();

   private final NewHighLevelControllerStateChangeStatusMessage highLevelStateChangeStatusMessage = new NewHighLevelControllerStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);

   public NewHumanoidHighLevelControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                                WholeBodyControllerCore controllerCore, NewHighLevelControllerStates initialControllerState,
                                                YoEnum<NewHighLevelControllerStates> requestedHighLevelControllerState,
                                                ArrayList<NewHighLevelControllerState> highLevelControllerStates,
                                                HashMap<NewHighLevelControllerState, ArrayList<StateTransition<NewHighLevelControllerStates>>> highLevelControllerTransitions,
                                                HighLevelHumanoidControllerToolbox controllerToolbox, CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      YoDouble yoTime = controllerToolbox.getYoTime();
      this.controllerCoreOutput = controllerCoreOutput;
      this.controllerCore = controllerCore;
      this.requestedHighLevelControllerState = requestedHighLevelControllerState;
      this.requestedHighLevelControllerState.set(initialControllerState);

      this.stateMachine = setUpStateMachine(highLevelControllerStates, highLevelControllerTransitions, yoTime, registry);

      isListeningToHighLevelStateMessage.set(true);

      for (int i = 0; i < highLevelControllerStates.size(); i++)
      {
         this.registry.addChild(highLevelControllerStates.get(i).getYoVariableRegistry());
      }
      this.initialControllerState = initialControllerState;
      this.controllerToolbox = controllerToolbox;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.registry.addChild(controllerToolbox.getYoVariableRegistry());

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

   public void setFallbackControllerForFailure(NewHighLevelControllerStates fallbackControllerState)
   {
      fallbackControllerForFailureReference.set(fallbackControllerState);
   }

   private GenericStateMachine<NewHighLevelControllerStates, NewHighLevelControllerState> setUpStateMachine(ArrayList<NewHighLevelControllerState> highLevelControllerStates,
                                                                                                            HashMap<NewHighLevelControllerState, ArrayList<StateTransition<NewHighLevelControllerStates>>> highLevelControllerTransitions,
                                                                                                            YoDouble yoTime, YoVariableRegistry registry)
   {
      GenericStateMachine<NewHighLevelControllerStates, NewHighLevelControllerState> highLevelStateMachine = new GenericStateMachine<>("highLevelControllerState", "switchTimeName",
                                                                                                                                       NewHighLevelControllerStates.class, yoTime, registry);

      for (int stateIndex = 0; stateIndex < highLevelControllerStates.size(); stateIndex++)
      {
         // set the controller core output
         NewHighLevelControllerState highLevelControllerState = highLevelControllerStates.get(stateIndex);
         highLevelControllerState.setControllerCoreOutput(controllerCoreOutput);

         // set the controller state transitions
         ArrayList<StateTransition<NewHighLevelControllerStates>> stateTransitions = highLevelControllerTransitions.get(highLevelControllerState);
         for (int transitionIndex = 0; transitionIndex < stateTransitions.size(); transitionIndex++)
            highLevelControllerState.addStateTransition(stateTransitions.get(transitionIndex));

         // add the controller to the state machine
         highLevelStateMachine.addState(highLevelControllerStates.get(stateIndex));
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

   public void initialize()
   {
      controllerCore.initialize();
      controllerToolbox.initialize();
      stateMachine.setCurrentState(initialControllerState);
   }

   public void doControl()
   {
      if (isListeningToHighLevelStateMessage.getBooleanValue())
      {
         if (commandInputManager.isNewCommandAvailable(HighLevelStateCommand.class))
         {
            requestedHighLevelControllerState.set(commandInputManager.pollNewestCommand(NewHighLevelStateCommand.class).getHighLevelState());
         }
      }

      stateMachine.checkTransitionConditions();
      highLevelControllerTimer.startMeasurement();
      stateMachine.doAction();
      highLevelControllerTimer.stopMeasurement();

      reportDesiredCenterOfPressureForEstimator();
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

   // FIXME should not be able to transition to all controllers

   /**
    * Adds a new high level controller to the controller manager state machine.
    * @param highLevelControllerState
    * @param transitionRequested
    */
   public void addHighLevelController(NewHighLevelControllerState highLevelControllerState, boolean transitionRequested)
   {
      highLevelControllerState.setControllerCoreOutput(controllerCoreOutput);

      // Enable transition between every existing state of the state machine
      for (NewHighLevelControllerStates stateEnum : NewHighLevelControllerStates.values)
      {
         FinishableState<NewHighLevelControllerStates> otherHighLevelState = stateMachine.getState(stateEnum);
         if (otherHighLevelState == null)
            continue;

         StateMachineTools.addRequestedStateTransition(requestedHighLevelControllerState, false, otherHighLevelState, highLevelControllerState);
         StateMachineTools.addRequestedStateTransition(requestedHighLevelControllerState, false, highLevelControllerState, otherHighLevelState);
      }

      this.stateMachine.addState(highLevelControllerState);
      this.registry.addChild(highLevelControllerState.getYoVariableRegistry());

      if (transitionRequested)
         requestedHighLevelControllerState.set(highLevelControllerState.getStateEnum());
   }

   public NewHighLevelControllerStates getCurrentHighLevelState()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return this.getClass().getSimpleName();
   }

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

}
