package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controlModules.flight.PelvisControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions.FlightToLandingCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions.LandingToStandingCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions.StandingToTakeOffCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions.TakeOffToFlightCondition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.AbstractJumpingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.FlightState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.LandingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.StandingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.TakeOffState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class JumpHighLevelHumanoidController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GenericStateMachine<JumpStateEnum, AbstractJumpingState> stateMachine;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final JumpControllerParameters jumpControllerParameters;
   private final JumpControlManagerFactory jumpControlManagerFactory;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final FullHumanoidRobotModel fullRobotModel;

   private final CentroidalMomentumManager centroidalMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final PelvisControlManager pelvisControlManager;
   private final FeetJumpManager feetManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();
   private final List<RigidBodyControlManager> bodyManagers = new ArrayList<>();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(PlaneContactStateCommand.class);
   private final Map<String, RigidBodyControlManager> rigidBodyManagersByName = new HashMap<>();
   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final YoEnum<JumpStateEnum> stateEnum;
   private final List<JumpControlManagerInterface> controlManagerList = new ArrayList<>();

   public JumpHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                          WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox,
                                          JumpControllerParameters jumpingControlParameters, JumpControlManagerFactory jumpControlManagerFactory,
                                          YoVariableRegistry parentRegistry)
   {
      String namePrefix = "Jump";
      this.stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", JumpStateEnum.class, controllerToolbox.getYoTime(),
                                                    registry);
      this.stateEnum = stateMachine.getStateYoVariable();

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      this.jumpControllerParameters = jumpingControlParameters;
      this.momentumOptimizationSettings = jumpingControlParameters.getMomentumOptimizationSettings();

      this.controlCoreToolbox = controlCoreToolbox;
      this.controllerToolbox = controllerToolbox;
      this.jumpControlManagerFactory = jumpControlManagerFactory;

      this.centroidalMomentumManager = jumpControlManagerFactory.getOrCreateCentroidalMomentumManager();
      controlManagerList.add(centroidalMomentumManager);
      this.gravityCompensationManager = jumpControlManagerFactory.getOrCreateGravityCompensationManager();
      controlManagerList.add(gravityCompensationManager);
      this.feetManager = jumpControlManagerFactory.getOrCreateFeetManager();
      controlManagerList.add(feetManager);
      this.pelvisControlManager = jumpControlManagerFactory.getOrCreatePelvisControlManager();
      controlManagerList.add(pelvisControlManager);

      this.fullRobotModel = controllerToolbox.getFullRobotModel();
      this.footSwitches = controllerToolbox.getFootSwitches();

      Collection<ReferenceFrame> trajectoryControlFrames = controllerToolbox.getTrajectoryFrames();
      RigidBody chest = fullRobotModel.getChest();
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      RigidBody head = fullRobotModel.getHead();
      ReferenceFrame headFrame = head.getBodyFixedFrame();

      RigidBody pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      for (RobotSide side : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(side);
         if (hand != null)
         {
            ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(side);
            RigidBodyControlManager handManager = jumpControlManagerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestFrame,
                                                                                                        trajectoryControlFrames);
            handManagers.put(side, handManager);
            bodyManagers.add(handManager);
         }
      }

      if (chest != null)
      {
         RigidBodyControlManager chestManager = jumpControlManagerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestFrame, pelvisFrame,
                                                                                                      trajectoryControlFrames);
         bodyManagers.add(chestManager);
      }
      if (head != null)
      {
         RigidBodyControlManager headManager = jumpControlManagerFactory.getOrCreateRigidBodyManager(head, chest, headFrame, chestFrame,
                                                                                                     trajectoryControlFrames);
         bodyManagers.add(headManager);
      }

      for (RigidBodyControlManager manager : bodyManagers)
      {
         if (manager == null)
         {
            continue;
         }
         rigidBodyManagersByName.put(manager.getControllerBodyName(), manager);
      }
      setupStateMachine();
      initializeManagers();
      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      StandingState standingState = new StandingState(centroidalMomentumManager, gravityCompensationManager, pelvisControlManager, handManagers, feetManager,
                                                      rigidBodyManagersByName, fullRobotModel);
      StandingToTakeOffCondition standingToTakeOffCondition = new StandingToTakeOffCondition(standingState);
      standingState.addStateTransition(JumpStateEnum.TAKE_OFF, standingToTakeOffCondition);
      stateMachine.addState(standingState);

      TakeOffState takeOffState = new TakeOffState(centroidalMomentumManager, gravityCompensationManager, pelvisControlManager, handManagers, feetManager,
                                                   rigidBodyManagersByName, fullRobotModel);
      TakeOffToFlightCondition takeOffToFlightCondition = new TakeOffToFlightCondition(takeOffState, footSwitches);
      takeOffState.addStateTransition(JumpStateEnum.FLIGHT, takeOffToFlightCondition);
      stateMachine.addState(takeOffState);

      FlightState flightState = new FlightState(controlCoreToolbox, controllerToolbox, centroidalMomentumManager, gravityCompensationManager, handManagers,
                                                feetManager, rigidBodyManagersByName);
      FlightToLandingCondition flightToLandingCondition = new FlightToLandingCondition(footSwitches);
      flightState.addStateTransition(JumpStateEnum.LANDING, flightToLandingCondition);
      stateMachine.addState(flightState);

      LandingState landingState = new LandingState();
      LandingToStandingCondition landingToStandingCondition = new LandingToStandingCondition(landingState);
      landingState.addStateTransition(JumpStateEnum.STANDING, landingToStandingCondition);
      stateMachine.addState(landingState);
   }

   public void doAction()
   {
      controllerToolbox.update();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      submitControllerCommands();
   }

   private void submitControllerCommands()
   {
      planeContactStateCommandPool.clear();
      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);

      for (int i = 0; i < controlManagerList.size(); i++)
      {
         JumpControlManagerInterface controlManager = controlManagerList.get(i);
         controllerCoreCommand.addInverseDynamicsCommand(controlManager.getInverseDynamicsCommand());
         controllerCoreCommand.addFeedbackControlCommand(controlManager.getFeedbackControlCommand());
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setUseHighCoPDamping(false);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }
      for (int i = 0; i < bodyManagers.size(); i++)
      {
         RigidBodyControlManager manager = bodyManagers.get(i);
         if (manager != null)
         {
            controllerCoreCommand.addFeedbackControlCommand(manager.getFeedbackControlCommand());
            controllerCoreCommand.addInverseDynamicsCommand(manager.getInverseDynamicsCommand());
         }
      }
   }

   public void initialize()
   {
      stateMachine.setCurrentState(JumpStateEnum.STANDING);
      controllerCoreCommand.requestReinitialization();
      controllerToolbox.initialize();

      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_ZERO);

         LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
         for (int i = 0; i < legJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getLegJoint(robotSide, legJointNames[i]), PrivilegedConfigurationOption.AT_ZERO);
      }
   }

   private void initializeManagers()
   {
      this.centroidalMomentumManager.setOptimizationWeights(momentumOptimizationSettings.getAngularMomentumWeight(),
                                                            momentumOptimizationSettings.getLinearMomentumWeight());
      this.centroidalMomentumManager.initialize();
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
