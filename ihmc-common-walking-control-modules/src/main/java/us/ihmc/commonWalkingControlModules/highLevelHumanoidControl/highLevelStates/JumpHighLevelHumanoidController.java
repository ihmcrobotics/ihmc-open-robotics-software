package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.AbstractJumpingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlightState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpLoadingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpUnloadingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.LandingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.TakeOffState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JumpHighLevelHumanoidController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GenericStateMachine<JumpStateEnum, AbstractJumpingState> stateMachine;
   
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   
   public JumpHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                             HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpingControlParameters,
                                             JumpControlManagerFactory jumpingControlManagerFactory)
   {
      String namePrefix = "jump";
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", JumpStateEnum.class, controllerToolbox.getYoTime(), registry);
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      
      setupStateMachine();
   }

   private void setupStateMachine()
   {
      TakeOffState takeOffState = new TakeOffState();
      JumpUnloadingState unloadingState = new JumpUnloadingState();
      FlightState flightState = new FlightState();
      JumpLoadingState loadingState = new JumpLoadingState();
      LandingState landingState = new LandingState();
      stateMachine.addState(takeOffState);
      stateMachine.addState(unloadingState);
      stateMachine.addState(flightState);
      stateMachine.addState(loadingState);
      stateMachine.addState(landingState);
      
      
   }
}
