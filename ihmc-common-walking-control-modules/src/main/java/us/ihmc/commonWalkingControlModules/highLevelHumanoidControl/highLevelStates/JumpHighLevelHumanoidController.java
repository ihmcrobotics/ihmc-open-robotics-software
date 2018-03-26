package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.AbstractJumpingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.FlightState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
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
   private final JumpControlManagerFactory jumpControlManagerFactory;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final FullHumanoidRobotModel fullRobotModel;

   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();
   private final SideDependentList<RigidBodyControlManager> footManagers = new SideDependentList<>();
   private final List<RigidBodyControlManager> bodyManagers = new ArrayList<>();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(PlaneContactStateCommand.class);
   private final Map<String, RigidBodyControlManager> rigidBodyManagersByName = new HashMap<>();

   public JumpHighLevelHumanoidController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                          WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox,
                                          JumpControllerParameters jumpingControlParameters, JumpControlManagerFactory jumpingControlManagerFactory)
   {
      String namePrefix = "jump";
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", JumpStateEnum.class, controllerToolbox.getYoTime(), registry);
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      this.jumpControllerParameters = jumpingControlParameters;
      this.momentumOptimizationSettings = jumpingControlParameters.getMomentumOptimizationSettings();

      this.controlCoreToolbox = controlCoreToolbox;
      this.controllerToolbox = controllerToolbox;
      this.jumpControlManagerFactory = jumpingControlManagerFactory;
      this.wholeBodyMomentumManager = jumpingControlManagerFactory.getOrCreateWholeBodyMomentumManager();

      this.gravityCompensationManager = jumpingControlManagerFactory.getOrCreateGravityCompensationManager();
      
      this.fullRobotModel = controllerToolbox.getFullRobotModel();

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
            RigidBodyControlManager handManager = jumpingControlManagerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestFrame,
                                                                                                           trajectoryControlFrames);
            handManagers.put(side, handManager);
            bodyManagers.add(handManager);
         }

         RigidBody foot = fullRobotModel.getFoot(side);
         if (foot != null)
         {
            ReferenceFrame footControlFrame = fullRobotModel.getSoleFrame(side);
            RigidBodyControlManager footManager = jumpingControlManagerFactory.getOrCreateRigidBodyManager(foot, pelvis, footControlFrame, pelvisFrame,
                                                                                                           trajectoryControlFrames);
            footManagers.put(side, footManager);
            bodyManagers.add(footManager);
         }
      }

      if (chest != null)
      {
         RigidBodyControlManager chestManager = jumpingControlManagerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestFrame, pelvisFrame,
                                                                                                         trajectoryControlFrames);
         bodyManagers.add(chestManager);
      }
      if (head != null)
      {
         RigidBodyControlManager headManager = jumpingControlManagerFactory.getOrCreateRigidBodyManager(head, chest, headFrame, chestFrame,
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
   }

   // TODO Hacked for now to default to the flight state
   private void setupStateMachine()
   {
      FlightState flightState = new FlightState(controlCoreToolbox, controllerToolbox, wholeBodyMomentumManager, gravityCompensationManager, handManagers,
                                                footManagers, rigidBodyManagersByName);
      stateMachine.addState(flightState);
   }

   public void doAction()
   {
      for (RobotSide side : RobotSide.values)
         controllerToolbox.setFootContactStateFree(side);
      controllerToolbox.update();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      submitControllerCommands();
   }

   private void submitControllerCommands()
   {
      planeContactStateCommandPool.clear();
      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);

      controllerCoreCommand.addInverseDynamicsCommand(wholeBodyMomentumManager.getMomentumRateCommand());
      controllerCoreCommand.addInverseDynamicsCommand(gravityCompensationManager.getRootJointAccelerationCommand());

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
      stateMachine.setCurrentState(JumpStateEnum.FLIGHT);
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
      this.wholeBodyMomentumManager.setOptimizationWeights(momentumOptimizationSettings.getAngularMomentumWeight(),
                                                           momentumOptimizationSettings.getLinearMomentumWeight());
      this.wholeBodyMomentumManager.initialize(controlCoreToolbox.getTotalRobotMass());
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
