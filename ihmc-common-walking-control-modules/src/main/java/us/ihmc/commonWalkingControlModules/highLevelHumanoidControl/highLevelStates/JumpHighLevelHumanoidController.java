package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.WholeBodyMomentumManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.AbstractJumpingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlightState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.robotics.hyperCubeTree.ConstantResolutionProvider;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JumpHighLevelHumanoidController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GenericStateMachine<JumpStateEnum, AbstractJumpingState> stateMachine;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final JumpControllerParameters jumpControllerParameters;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyMomentumManager wholeBodyMomentumManager;
   //private final FeetJumpManager feetManager;
   
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(PlaneContactStateCommand.class);
   
   public JumpHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                             WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpingControlParameters,
                                             JumpControlManagerFactory jumpingControlManagerFactory)
   {
      String namePrefix = "jump";
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", JumpStateEnum.class, controllerToolbox.getYoTime(), registry);
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      
      this.jumpControllerParameters = jumpingControlParameters;
      this.momentumOptimizationSettings = jumpingControlParameters.getMomentumOptimizationSettings();
      
      this.controlCoreToolbox = controlCoreToolbox;
      this.controllerToolbox = controllerToolbox;
      this.wholeBodyMomentumManager = jumpingControlManagerFactory.getOrCreateWholeBodyMomentumManager();
      this.wholeBodyMomentumManager.setOptimizationWeights(momentumOptimizationSettings.getAngularMomentumWeight(), momentumOptimizationSettings.getLinearMomentumWeight());
      this.wholeBodyMomentumManager.setTotalRobotMass(controlCoreToolbox.getTotalRobotMass());
      //this.feetManager = jumpingControlManagerFactory.getOrCreateFeetJumpManager();
      setupStateMachine();
   }

   // TODO Hacked for now to default to the flight state
   private void setupStateMachine()
   {
      //FlightState flightState = new FlightState(controlCoreToolbox, controllerToolbox, wholeBodyMomentumManager, feetManager);
      FlightState flightState = new FlightState(controlCoreToolbox, controllerToolbox, wholeBodyMomentumManager, null);
      stateMachine.addState(flightState);
      stateMachine.setCurrentState(JumpStateEnum.FLIGHT);
   }

   public void doAction()
   {
      for(RobotSide side : RobotSide.values)
         controllerToolbox.setFootContactStateFree(side);
      controllerToolbox.update();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      submitControllerCommands();
   }

   private void submitControllerCommands()
   {
      planeContactStateCommandPool.clear();
      controllerCoreCommand.addInverseDynamicsCommand(wholeBodyMomentumManager.getMomentumRateCommand());
      //controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(RobotSide.LEFT));
      for (RobotSide robotSide : RobotSide.values)
      {
         //controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotSide));
         //controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(robotSide));

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setUseHighCoPDamping(false);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }
   }

   public void initialize()
   {
      controllerCoreCommand.requestReinitialization();
      controllerToolbox.initialize();
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
