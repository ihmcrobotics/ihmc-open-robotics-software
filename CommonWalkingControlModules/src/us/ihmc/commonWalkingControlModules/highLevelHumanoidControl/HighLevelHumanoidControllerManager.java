package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.time.ExecutionTimer;

public class HighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final GenericStateMachine<HighLevelState, HighLevelBehavior> stateMachine;
   private final HighLevelState initialBehavior;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final EnumYoVariable<HighLevelState> requestedHighLevelState = new EnumYoVariable<HighLevelState>("requestedHighLevelState", registry,
         HighLevelState.class, true);

   private final BooleanYoVariable isListeningToHighLevelStateMessage = new BooleanYoVariable("isListeningToHighLevelStateMessage", registry);

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final ControllerCoreOutputReadOnly controllerCoreOutput;
   private final WholeBodyControllerCore controllerCore;

   private final AtomicReference<HighLevelState> fallbackControllerForFailureReference = new AtomicReference<>();

   private final HighLevelStateChangeStatusMessage highLevelStateChangeStatusMessage = new HighLevelStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);
   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   public HighLevelHumanoidControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
         WholeBodyControllerCore controllerCore, HighLevelState initialBehavior, ArrayList<HighLevelBehavior> highLevelBehaviors,
         HighLevelHumanoidControllerToolbox controllerToolbox, CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
         ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      this.controllerCoreOutput = controllerCoreOutput;
      this.controllerCore = controllerCore;

      this.stateMachine = setUpStateMachine(highLevelBehaviors, yoTime, registry);
      requestedHighLevelState.set(initialBehavior);

      isListeningToHighLevelStateMessage.set(true);

      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         this.registry.addChild(highLevelBehaviors.get(i).getYoVariableRegistry());
      }
      this.initialBehavior = initialBehavior;
      this.controllerToolbox = controllerToolbox;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.registry.addChild(controllerToolbox.getYoVariableRegistry());

      controllerToolbox.attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2d fallingDirection)
         {
            HighLevelState fallbackController = fallbackControllerForFailureReference.get();
            if (fallbackController != null)
               requestedHighLevelState.set(fallbackController);
         }
      });
   }

   public void setFallbackControllerForFailure(HighLevelState fallbackController)
   {
      fallbackControllerForFailureReference.set(fallbackController);
   }

   private GenericStateMachine<HighLevelState, HighLevelBehavior> setUpStateMachine(ArrayList<HighLevelBehavior> highLevelBehaviors, DoubleYoVariable yoTime,
         YoVariableRegistry registry)
   {
      GenericStateMachine<HighLevelState, HighLevelBehavior> highLevelStateMachine = new GenericStateMachine<>("highLevelState", "switchTimeName",
            HighLevelState.class, yoTime, registry);

      // Enable transition between every existing state of the state machine
      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         HighLevelBehavior highLevelStateA = highLevelBehaviors.get(i);
         highLevelStateA.setControllerCoreOutput(controllerCoreOutput);

         for (int j = 0; j < highLevelBehaviors.size(); j++)
         {
            HighLevelBehavior highLevelStateB = highLevelBehaviors.get(j);

            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateA, highLevelStateB);
            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateB, highLevelStateA);
         }
      }

      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         highLevelStateMachine.addState(highLevelBehaviors.get(i));
      }

      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<HighLevelState>()
      {
         @Override
         public void stateChanged(State<HighLevelState> oldState, State<HighLevelState> newState, double time)
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

   public void requestHighLevelState(HighLevelState requestedHighLevelState)
   {
      this.requestedHighLevelState.set(requestedHighLevelState);
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
            requestedHighLevelState.set(commandInputManager.pollNewestCommand(HighLevelStateCommand.class).getHighLevelState());
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

   private final SideDependentList<FramePoint2d> desiredFootCoPs = new SideDependentList<FramePoint2d>(new FramePoint2d(), new FramePoint2d());

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

   public void addHighLevelBehavior(HighLevelBehavior highLevelBehavior, boolean transitionRequested)
   {
      highLevelBehavior.setControllerCoreOutput(controllerCoreOutput);

      // Enable transition between every existing state of the state machine
      for (HighLevelState stateEnum : HighLevelState.values)
      {
         FinishableState<HighLevelState> otherHighLevelState = stateMachine.getState(stateEnum);
         if (otherHighLevelState == null)
            continue;

         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, otherHighLevelState, highLevelBehavior);
         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelBehavior, otherHighLevelState);
      }

      this.stateMachine.addState(highLevelBehavior);
      this.registry.addChild(highLevelBehavior.getYoVariableRegistry());

      if (transitionRequested)
         requestedHighLevelState.set(highLevelBehavior.getStateEnum());
   }

   public HighLevelState getCurrentHighLevelState()
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

}
