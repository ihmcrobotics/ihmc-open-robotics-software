package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
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
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.NewHighLevelControllerStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelStates;
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
import java.util.concurrent.atomic.AtomicReference;

public class NewHighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final GenericStateMachine<NewHighLevelStates, NewHighLevelControllerState> stateMachine;
   private final NewHighLevelStates initialBehavior;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoEnum<NewHighLevelStates> requestedHighLevelControllerState = new YoEnum<NewHighLevelStates>("requestedHighLevelControllerState", registry,
         NewHighLevelStates.class, true);

   private final YoBoolean isListeningToHighLevelStateMessage = new YoBoolean("isListeningToHighLevelStateMessage", registry);

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final ControllerCoreOutputReadOnly controllerCoreOutput;
   private final WholeBodyControllerCore controllerCore;

   private final AtomicReference<NewHighLevelStates> fallbackControllerForFailureReference = new AtomicReference<>();

   private final NewHighLevelControllerStateChangeStatusMessage highLevelStateChangeStatusMessage = new NewHighLevelControllerStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);
   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   public NewHighLevelHumanoidControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                                WholeBodyControllerCore controllerCore, NewHighLevelStates initialBehavior,
                                                ArrayList<NewHighLevelControllerState> highLevelStates, HighLevelHumanoidControllerToolbox controllerToolbox,
                                                CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      YoDouble yoTime = controllerToolbox.getYoTime();
      this.controllerCoreOutput = controllerCoreOutput;
      this.controllerCore = controllerCore;

      this.stateMachine = setUpStateMachine(highLevelStates, yoTime, registry);
      requestedHighLevelControllerState.set(initialBehavior);

      isListeningToHighLevelStateMessage.set(true);

      for (int i = 0; i < highLevelStates.size(); i++)
      {
         this.registry.addChild(highLevelStates.get(i).getYoVariableRegistry());
      }
      this.initialBehavior = initialBehavior;
      this.controllerToolbox = controllerToolbox;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.registry.addChild(controllerToolbox.getYoVariableRegistry());

      controllerToolbox.attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2D fallingDirection)
         {
            NewHighLevelStates fallbackController = fallbackControllerForFailureReference.get();
            if (fallbackController != null)
               requestedHighLevelControllerState.set(fallbackController);
         }
      });
   }

   public void setFallbackControllerForFailure(NewHighLevelStates fallbackController)
   {
      fallbackControllerForFailureReference.set(fallbackController);
   }

   /** FIXME this should not have transitions to every state **/
   private GenericStateMachine<NewHighLevelStates, NewHighLevelControllerState> setUpStateMachine(ArrayList<NewHighLevelControllerState> highLevelControllerStates,
                                                                                                  YoDouble yoTime, YoVariableRegistry registry)
   {
      GenericStateMachine<NewHighLevelStates, NewHighLevelControllerState> highLevelStateMachine = new GenericStateMachine<>("highLevelState", "switchTimeName",
            NewHighLevelStates.class, yoTime, registry);

      // Enable transition between every existing state of the state machine
      for (int i = 0; i < highLevelControllerStates.size(); i++)
      {
         NewHighLevelControllerState highLevelStateA = highLevelControllerStates.get(i);
         highLevelStateA.setControllerCoreOutput(controllerCoreOutput);

         for (int j = 0; j < highLevelControllerStates.size(); j++)
         {
            NewHighLevelControllerState highLevelStateB = highLevelControllerStates.get(j);

            StateMachineTools.addRequestedStateTransition(requestedHighLevelControllerState, false, highLevelStateA, highLevelStateB);
            StateMachineTools.addRequestedStateTransition(requestedHighLevelControllerState, false, highLevelStateB, highLevelStateA);
         }
      }

      for (int i = 0; i < highLevelControllerStates.size(); i++)
      {
         highLevelStateMachine.addState(highLevelControllerStates.get(i));
      }

      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<NewHighLevelStates>()
      {
         @Override
         public void stateChanged(State<NewHighLevelStates> oldState, State<NewHighLevelStates> newState, double time)
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

   public void requestHighLevelControllerState(NewHighLevelStates requestedHighLevelControllerState)
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
      stateMachine.setCurrentState(initialBehavior);
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
      ControllerCoreCommand controllerCoreCommandList = stateMachine.getCurrentState().getControllerCoreCommand();
      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommandList);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();
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

   public void addHighLevelController(NewHighLevelControllerState highLevelControllerState, boolean transitionRequested)
   {
      highLevelControllerState.setControllerCoreOutput(controllerCoreOutput);

      // Enable transition between every existing state of the state machine
      for (NewHighLevelStates stateEnum : NewHighLevelStates.values)
      {
         FinishableState<NewHighLevelStates> otherHighLevelState = stateMachine.getState(stateEnum);
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

   public NewHighLevelStates getCurrentHighLevelState()
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
   public void warmup(int iterations, NewWalkingControllerState walkingState)
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
