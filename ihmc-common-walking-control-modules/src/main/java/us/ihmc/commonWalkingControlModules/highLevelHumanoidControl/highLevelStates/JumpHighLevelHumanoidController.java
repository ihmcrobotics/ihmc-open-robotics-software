package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.GravityCompensationManager;
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
   
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(PlaneContactStateCommand.class);

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
      this.wholeBodyMomentumManager.setOptimizationWeights(momentumOptimizationSettings.getAngularMomentumWeight(),
                                                           momentumOptimizationSettings.getLinearMomentumWeight());
      this.wholeBodyMomentumManager.setTotalRobotMass(controlCoreToolbox.getTotalRobotMass());

      this.gravityCompensationManager = jumpingControlManagerFactory.getOrCreateGravityCompensationManager();
      this.fullRobotModel = controllerToolbox.getFullRobotModel();

      Collection<ReferenceFrame> trajectoryControlFrames = controllerToolbox.getTrajectoryFrames();
      RigidBody chest = fullRobotModel.getChest();
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      RigidBody pelvis= fullRobotModel.getPelvis();
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
         }

         RigidBody foot = fullRobotModel.getFoot(side);
         if (foot != null)
         {
            ReferenceFrame footControlFrame = fullRobotModel.getSoleFrame(side);
            RigidBodyControlManager footManager = jumpingControlManagerFactory.getOrCreateRigidBodyManager(foot, pelvis, footControlFrame, pelvisFrame,
                                                                                                           trajectoryControlFrames);
            footManagers.put(side, footManager);
         }
      }

      setupStateMachine();
   }
   
   // TODO Hacked for now to default to the flight state
   private void setupStateMachine()
   {
      FlightState flightState = new FlightState(controlCoreToolbox, controllerToolbox, wholeBodyMomentumManager, gravityCompensationManager, handManagers,
                                                footManagers);
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
      controllerCoreCommand.addInverseDynamicsCommand(wholeBodyMomentumManager.getMomentumRateCommand());
      controllerCoreCommand.addInverseDynamicsCommand(wholeBodyMomentumManager.getCoMAccelerationCommand());
      controllerCoreCommand.addInverseDynamicsCommand(gravityCompensationManager.getRootJointAccelerationCommand());

      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setUseHighCoPDamping(false);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);

         RigidBodyControlManager handManager = handManagers.get(robotSide);
         if(handManager != null)
         {
            controllerCoreCommand.addFeedbackControlCommand(handManager.getFeedbackControlCommand());
            controllerCoreCommand.addInverseDynamicsCommand(handManager.getInverseDynamicsCommand());
         }
         RigidBodyControlManager footManager = footManagers.get(robotSide);
         if(footManager != null)
         {
            controllerCoreCommand.addFeedbackControlCommand(footManager.getFeedbackControlCommand());
            controllerCoreCommand.addInverseDynamicsCommand(footManager.getInverseDynamicsCommand());
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
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);

         LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
         for (int i = 0; i < legJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getLegJoint(robotSide, legJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);
      }

   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
