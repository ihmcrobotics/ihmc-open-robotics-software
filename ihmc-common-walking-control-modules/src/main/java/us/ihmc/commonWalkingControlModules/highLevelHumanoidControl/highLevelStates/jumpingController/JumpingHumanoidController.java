package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.JumpingMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointLoadStatusProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ParameterizedControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class JumpingHumanoidController implements JointLoadStatusProvider
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble yoTime;

   private final JumpingControlManagerFactory managerFactory;

   private final JumpingPelvisOrientationManager pelvisOrientationManager;
   private final JumpingFeetManager feetManager;
   private final JumpingBalanceManager balanceManager;

   private final ArrayList<RigidBodyControlManager> bodyManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> bodyManagerByJointName = new HashMap<>();
   private final SideDependentList<Set<String>> legJointNames = new SideDependentList<>();

   private final FullHumanoidRobotModel fullRobotModel;
   private final JumpingControllerToolbox controllerToolbox;

   private final StateMachine<JumpingStateEnum, JumpingState> stateMachine;

   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();
   private final YoBoolean limitCommandSent = new YoBoolean("limitCommandSent", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final WalkingControllerParameters walkingControllerParameters;
   private final ParameterizedControllerCoreOptimizationSettings controllerCoreOptimizationSettings;

   private boolean firstTick = true;

   public JumpingHumanoidController(JumpingControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
                                    CoPTrajectoryParameters copTrajectoryParameters, JumpingControllerToolbox controllerToolbox)
   {
      this.managerFactory = managerFactory;
      this.walkingControllerParameters = walkingControllerParameters;

      // Getting parameters from the JumpingControllerToolbox
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      yoTime = controllerToolbox.getYoTime();

      OneDoFJointBasics[] allOneDoFJoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);

      this.pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      this.feetManager = managerFactory.getOrCreateFeetManager();

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();

      ReferenceFrame chestBodyFrame = null;
      if (chest != null)
      {
         chestBodyFrame = chest.getBodyFixedFrame();
         RigidBodyControlManager chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisZUpFrame);
         bodyManagers.add(chestManager);
      }

      if (head != null)
      {
         ReferenceFrame headBodyFrame = head.getBodyFixedFrame();
         RigidBodyControlManager headManager = managerFactory.getOrCreateRigidBodyManager(head, chest, headBodyFrame, chestBodyFrame);
         bodyManagers.add(headManager);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
         bodyManagers.add(handManager);
      }

      for (RigidBodyControlManager manager : bodyManagers)
      {
         if (manager == null)
         {
            continue;
         }
         Arrays.asList(manager.getControlledJoints()).forEach(joint -> bodyManagerByJointName.put(joint.getName(), manager));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         OneDoFJointBasics[] legJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(pelvis, foot), OneDoFJointBasics.class);
         Set<String> jointNames = new HashSet<>();
         Arrays.asList(legJoints).forEach(legJoint -> jointNames.add(legJoint.getName()));
         legJointNames.put(robotSide, jointNames);
      }

      balanceManager = new JumpingBalanceManager(controllerToolbox, copTrajectoryParameters, registry);

      failureDetectionControlModule = new WalkingFailureDetectionControlModule(controllerToolbox.getContactableFeet(), registry);

      stateMachine = setupStateMachine(balanceManager);

      String[] jointNamesRestrictiveLimits = walkingControllerParameters.getJointsWithRestrictiveLimits();
      JointLimitParameters limitParameters = walkingControllerParameters.getJointLimitParametersForJointsWithRestictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimit = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFJoints,
                                                                                                                        jointNamesRestrictiveLimits),
                                                                                         OneDoFJointBasics.class);
      for (OneDoFJointBasics joint : jointsWithRestrictiveLimit)
      {
         if (limitParameters == null)
         {
            throw new RuntimeException("Must define joint limit parameters if using joints with restrictive limits.");
         }
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }

      ControllerCoreOptimizationSettings defaultControllerCoreOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      controllerCoreOptimizationSettings = new ParameterizedControllerCoreOptimizationSettings(defaultControllerCoreOptimizationSettings, registry);
   }

   private StateMachine<JumpingStateEnum, JumpingState> setupStateMachine(JumpingBalanceManager balanceManager)
   {
      StateMachineFactory<JumpingStateEnum, JumpingState> factory = new StateMachineFactory<>(JumpingStateEnum.class);
      factory.setNamePrefix("jumping").setRegistry(registry).buildYoClock(yoTime);

      JumpingStandingState standingState = new JumpingStandingState(controllerToolbox, managerFactory,
                                                                    balanceManager, failureDetectionControlModule, registry);
      TransferToJumpingStandingState toStandingState = new TransferToJumpingStandingState(controllerToolbox, managerFactory,
                                                                                          balanceManager, failureDetectionControlModule, registry);
      factory.addState(JumpingStateEnum.TO_STANDING, toStandingState);
      factory.addState(JumpingStateEnum.STANDING, standingState);

      factory.addDoneTransition(JumpingStateEnum.TO_STANDING, JumpingStateEnum.STANDING);

      // Update the previous state info for each state using state changed listeners.
      factory.getRegisteredStates().forEach(state -> factory.addStateChangedListener((from, to) -> state.setPreviousJumpingStateEnum(from)));
      factory.addStateChangedListener(controllerToolbox::reportControllerStateChangeToListeners);

      return factory.build(JumpingStateEnum.TO_STANDING);
   }

   public void initialize()
   {
      controllerCoreCommand.requestReinitialization();
      controllerToolbox.initialize();
      managerFactory.initializeManagers();

      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_MID_RANGE);
      }

      initializeContacts();

      stateMachine.resetToInitialState();

      initializeManagers();

      firstTick = true;
   }

   private void initializeManagers()
   {
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      pelvisOrientationManager.setTrajectoryTime(stepTime);

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
            bodyManager.initialize();
      }


      pelvisOrientationManager.initialize();
//      balanceManager.initialize();  // already initialized, so don't run it again, or else the state machine gets reset.
      feetManager.initialize();
   }

   private void initializeContacts()
   {
      controllerToolbox.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.resetFootPlaneContactPoint(robotSide);
         feetManager.setFlatFootContactState(robotSide);
      }
   }

   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);
   private final FramePoint2D capturePoint2d = new FramePoint2D();

   public void doAction()
   {
      updateFailureDetection();

      // Do transitions will request ICP planner updates.
      if (!firstTick) // this avoids doing two transitions in a single tick if the initialize reset the state machine.
         stateMachine.doTransitions();
      // Do action is relying on the ICP plan being valid.
      stateMachine.doAction();

      updateManagers();

      handleChangeInContactState();

      submitControllerCoreCommands();

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.getFootContactState(robotSide).pollContactHasChangedNotification();
      }

      firstTick = false;
   }

   private void handleChangeInContactState()
   {
      boolean haveContactStatesChanged = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         if (contactState.peekContactHasChangedNotification())
            haveContactStatesChanged = true;
      }

      if (!haveContactStatesChanged)
         return;

      controllerToolbox.updateBipedSupportPolygons();
      balanceManager.computeICPPlan();
   }

   public void updateFailureDetection()
   {
      capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint2d, balanceManager.getDesiredICP());
   }

   public void updateManagers()
   {
      feetManager.compute();

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
            bodyManager.compute();
      }

      pelvisOrientationManager.compute();
      balanceManager.compute();
   }

   private void submitControllerCoreCommands()
   {
      planeContactStateCommandPool.clear();

      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      if (!limitCommandSent.getBooleanValue())
      {
         controllerCoreCommand.addInverseDynamicsCommand(jointLimitEnforcementMethodCommand);
         limitCommandSent.set(true);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotSide));
         controllerCoreCommand.addInverseDynamicsCommand(feetManager.getInverseDynamicsCommand(robotSide));

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }

      for (int managerIdx = 0; managerIdx < bodyManagers.size(); managerIdx++)
      {
         RigidBodyControlManager bodyManager = bodyManagers.get(managerIdx);
         if (bodyManager != null)
         {
            controllerCoreCommand.addFeedbackControlCommand(bodyManager.getFeedbackControlCommand());
            controllerCoreCommand.addInverseDynamicsCommand(bodyManager.getInverseDynamicsCommand());
         }
      }

      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationManager.getFeedbackControlCommand());

      controllerCoreCommand.addInverseDynamicsCommand(controllerCoreOptimizationSettings.getCommand());
   }

   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

   public JumpingMomentumRateControlModuleInput getJumpingMomentumRateControlModuleInput()
   {
      return balanceManager.getJumpingMomentumRateControlModuleInput();
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public boolean isJointLoadBearing(String jointName)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         boolean legLoaded = feetManager.getCurrentConstraintType(robotSide).isLoadBearing();
         if (legLoaded && legJointNames.get(robotSide).contains(jointName))
         {
            return true;
         }
      }

      RigidBodyControlManager manager = bodyManagerByJointName.get(jointName);
      if (manager != null && manager.isLoadBearing())
      {
         return true;
      }

      return false;
   }
}
