package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools.addRequestedStateTransition;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.HandUserControlModeState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.JointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LoadBearingHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.io.printing.PrintTools;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // JPratt. February 27, 2015: Added this since new Atlas was having trouble with network stuff.
   // It was sending 14,000 variables. This and others reduces it a bit when set to false.
   private static final boolean REGISTER_YOVARIABLES = true;

   private final YoVariableRegistry registry;

   private final GenericStateMachine<HandControlMode, HandControlState> stateMachine;

   private final TaskspaceHandControlState taskspaceControlState;
   private final JointSpaceHandControlState jointspaceControlState;
   private final LoadBearingHandControlState loadBearingControlState;
   private final HandUserControlModeState userControlModeState;

   private final EnumYoVariable<HandControlMode> requestedState;
   private final OneDoFJoint[] controlledJoints;

   private final Map<OneDoFJoint, BooleanYoVariable> areJointsEnabled;
   private final String name;
   private final RigidBody chest, hand;

   private final FullHumanoidRobotModel fullRobotModel;

   private final StateChangedListener<HandControlMode> stateChangedlistener;
   private final BooleanYoVariable hasHandPoseStatusBeenSent;

   private final BooleanYoVariable areAllArmJointEnabled;

   private final boolean isAtLeastOneJointPositionControlled;
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand;
   private final Map<OneDoFJoint, DoubleYoVariable> accelerationIntegrationAlphaPosition;
   private final Map<OneDoFJoint, DoubleYoVariable> accelerationIntegrationAlphaVelocity;
   private final Map<OneDoFJoint, DoubleYoVariable> accelerationIntegrationMaxPositionError;
   private final Map<OneDoFJoint, DoubleYoVariable> accelerationIntegrationMaxVelocity;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   private final ReferenceFrame chestFrame;
   private final ReferenceFrame handControlFrame;

   public HandControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox momentumBasedController, ArmControllerParameters armControlParameters,
         YoPIDGains jointspaceGains, YoSE3PIDGainsInterface taskspaceGains, YoVariableRegistry parentRegistry)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry;
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      if (REGISTER_YOVARIABLES)
      {
         yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
         parentRegistry.addChild(registry);
      }
      else
      {
         yoGraphicsListRegistry = null;
      }

      hasHandPoseStatusBeenSent = new BooleanYoVariable(namePrefix + "HasHandPoseStatusBeenSent", registry);
      hasHandPoseStatusBeenSent.set(false);

      areAllArmJointEnabled = new BooleanYoVariable(namePrefix + "AreAllArmJointEnabled", registry);
      areAllArmJointEnabled.set(true);

      stateChangedlistener = new StateChangedListener<HandControlMode>()
      {
         @Override
         public void stateChanged(State<HandControlMode> oldState, State<HandControlMode> newState, double time)
         {
            hasHandPoseStatusBeenSent.set(false);
         }
      };

      fullRobotModel = momentumBasedController.getFullRobotModel();
      hand = fullRobotModel.getHand(robotSide);
      chest = fullRobotModel.getChest();

      chestFrame = chest.getBodyFixedFrame();
      handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

      controlledJoints = ScrewTools.createOneDoFJointPath(chest, hand);

      requestedState = new EnumYoVariable<HandControlMode>(name + "RequestedState", "", registry, HandControlMode.class, true);
      requestedState.set(null);

      areJointsEnabled = new LinkedHashMap<>();
      for (OneDoFJoint oneDoFJoint : controlledJoints)
      {
         areJointsEnabled.put(oneDoFJoint, new BooleanYoVariable(namePrefix + oneDoFJoint.getName() + "IsEnabled", registry));
      }

      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      stateMachine = new GenericStateMachine<>(name, name + "SwitchTime", HandControlMode.class, yoTime, registry);

      RigidBody elevator = fullRobotModel.getElevator();
      String[] positionControlledJointNames = armControlParameters.getPositionControlledJointNames(robotSide);
      isAtLeastOneJointPositionControlled = positionControlledJointNames != null && positionControlledJointNames.length > 0;

      OneDoFJoint[] positionControlledJoints;
      if (isAtLeastOneJointPositionControlled)
         positionControlledJoints = ScrewTools.filterJoints(ScrewTools.findJointsWithNames(controlledJoints, positionControlledJointNames), OneDoFJoint.class);
      else
         positionControlledJoints = new OneDoFJoint[0];

      String stateNamePrefix = namePrefix + "Hand";

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      Map<BaseForControl, ReferenceFrame> baseForControlToReferenceFrameMap = new EnumMap<>(BaseForControl.class);
      baseForControlToReferenceFrameMap.put(BaseForControl.CHEST, chestFrame);
      baseForControlToReferenceFrameMap.put(BaseForControl.WALKING_PATH, referenceFrames.getMidFeetUnderPelvisFrame());
      baseForControlToReferenceFrameMap.put(BaseForControl.WORLD, worldFrame);

      Map<OneDoFJoint, Double> homeConfiguration = armControlParameters.getDefaultArmJointPositions(fullRobotModel, robotSide);

      jointspaceControlState = new JointSpaceHandControlState(stateNamePrefix, homeConfiguration, controlledJoints, jointspaceGains, registry);
      taskspaceControlState = new TaskspaceHandControlState(stateNamePrefix, elevator, hand, chest, taskspaceGains, baseForControlToReferenceFrameMap,
            yoGraphicsListRegistry, registry);
      userControlModeState = new HandUserControlModeState(stateNamePrefix, controlledJoints, momentumBasedController, registry);

      if (isAtLeastOneJointPositionControlled)
      {
         // TODO Not implemented for position control.
         loadBearingControlState = null;

         jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
         lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

         lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(positionControlledJoints);
         lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(positionControlledJoints, LowLevelJointControlMode.POSITION_CONTROL);

         for (OneDoFJoint positionControlledJoint : positionControlledJoints)
            jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(positionControlledJoint);

         Map<ArmJointName, DoubleYoVariable> alphaPosition = armControlParameters.getOrCreateAccelerationIntegrationAlphaPosition(parentRegistry);
         Map<ArmJointName, DoubleYoVariable> alphaVelocity = armControlParameters.getOrCreateAccelerationIntegrationAlphaVelocity(parentRegistry);
         Map<ArmJointName, DoubleYoVariable> alphaMaxPositionError = armControlParameters.getOrCreateAccelerationIntegrationMaxPositionError(parentRegistry);
         Map<ArmJointName, DoubleYoVariable> alphaMaxVelocity = armControlParameters.getOrCreateAccelerationIntegrationMaxVelocity(parentRegistry);

         accelerationIntegrationAlphaPosition = new HashMap<>();
         accelerationIntegrationAlphaVelocity = new HashMap<>();
         accelerationIntegrationMaxPositionError = new HashMap<>();
         accelerationIntegrationMaxVelocity = new HashMap<>();

         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();

         for (ArmJointName armJointName : armJointNames)
         {
            OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
            if (alphaPosition != null && alphaPosition.containsKey(armJointName))
               accelerationIntegrationAlphaPosition.put(armJoint, alphaPosition.get(armJointName));
            if (alphaVelocity != null && alphaVelocity.containsKey(armJointName))
               accelerationIntegrationAlphaVelocity.put(armJoint, alphaVelocity.get(armJointName));
            if (alphaMaxPositionError != null && alphaMaxPositionError.containsKey(armJointName))
               accelerationIntegrationMaxPositionError.put(armJoint, alphaMaxPositionError.get(armJointName));
            if (alphaMaxVelocity != null && alphaMaxVelocity.containsKey(armJointName))
               accelerationIntegrationMaxVelocity.put(armJoint, alphaMaxVelocity.get(armJointName));
         }
      }
      else
      {
         loadBearingControlState = new LoadBearingHandControlState(stateNamePrefix, HandControlMode.LOAD_BEARING, robotSide, momentumBasedController, elevator,
               hand, registry);
         jointAccelerationIntegrationCommand = null;
         lowLevelOneDoFJointDesiredDataHolder = null;
         accelerationIntegrationAlphaPosition = null;
         accelerationIntegrationAlphaVelocity = null;
         accelerationIntegrationMaxPositionError = null;
         accelerationIntegrationMaxVelocity = null;
      }

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      List<HandControlState> allStates = new ArrayList<>();
      allStates.add(jointspaceControlState);
      allStates.add(taskspaceControlState);
      allStates.add(userControlModeState);
      if (loadBearingControlState != null)
         allStates.add(loadBearingControlState);

      for (HandControlState fromState : allStates)
      {
         for (HandControlState toState : allStates)
         {
            addRequestedStateTransition(requestedState, false, fromState, toState);
         }
         stateMachine.addState(fromState);
      }

      stateMachine.attachStateChangedListener(stateChangedlistener);
   }

   public void setTaskspaceWeight(double weight)
   {
      taskspaceControlState.setWeight(weight);
   }

   public void setTaskspaceWeights(Vector3D angularWeight, Vector3D linearWeight)
   {
      taskspaceControlState.setWeights(angularWeight, linearWeight);
   }

   public void setJointspaceWeight(double weight)
   {
      jointspaceControlState.setWeight(weight);
   }

   public void setUserModeWeight(double weight)
   {
      userControlModeState.setWeight(weight);
   }

   public void initialize()
   {
      holdPositionInJointspace();
   }

   public void doControl()
   {
      boolean isAtLeastOneJointDisabled = checkIfAtLeastOneJointIsDisabled();

      if (isAtLeastOneJointDisabled && areAllArmJointEnabled.getBooleanValue())
      {
         if (ManipulationControlModule.HOLD_POSE_IN_JOINT_SPACE)
            holdPositionInJointspace();
         else
            holdPositionInChest();

         areAllArmJointEnabled.set(false);
      }
      else if (!isAtLeastOneJointDisabled)
      {
         areAllArmJointEnabled.set(true);
      }

      if (stateMachine.getCurrentState().isAbortRequested())
      {
         holdPositionInJointspace();
      }

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      updateAccelerationIntegrationParameters();

      updateInverseDynamicsCommandList();
   }

   private boolean checkIfAtLeastOneJointIsDisabled()
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         areJointsEnabled.get(controlledJoints[i]).set(controlledJoints[i].isEnabled());
         if (!controlledJoints[i].isEnabled())
            return true;
      }
      return false;
   }

   private void updateAccelerationIntegrationParameters()
   {
      if (jointAccelerationIntegrationCommand == null)
         return;

      for (int i = 0; i < jointAccelerationIntegrationCommand.getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         OneDoFJoint jointToComputeDesiredPositionFor = jointAccelerationIntegrationCommand.getJointToComputeDesiredPositionFor(i);
         double alphaPosition = Double.NaN;
         double alphaVelocity = Double.NaN;
         if (accelerationIntegrationAlphaPosition != null && accelerationIntegrationAlphaPosition.containsKey(jointToComputeDesiredPositionFor))
            alphaPosition = accelerationIntegrationAlphaPosition.get(jointToComputeDesiredPositionFor).getDoubleValue();
         if (accelerationIntegrationAlphaVelocity != null && accelerationIntegrationAlphaVelocity.containsKey(jointToComputeDesiredPositionFor))
            alphaVelocity = accelerationIntegrationAlphaVelocity.get(jointToComputeDesiredPositionFor).getDoubleValue();
         jointAccelerationIntegrationCommand.setJointAlphas(i, alphaPosition, alphaVelocity);

         double maxPositionError = Double.NaN;
         double maxVelocity = Double.NaN;
         if (accelerationIntegrationMaxPositionError != null && accelerationIntegrationMaxPositionError.containsKey(jointToComputeDesiredPositionFor))
            maxPositionError = accelerationIntegrationMaxPositionError.get(jointToComputeDesiredPositionFor).getDoubleValue();
         if (accelerationIntegrationMaxVelocity != null && accelerationIntegrationMaxVelocity.containsKey(jointToComputeDesiredPositionFor))
            maxVelocity = accelerationIntegrationMaxVelocity.get(jointToComputeDesiredPositionFor).getDoubleValue();
         jointAccelerationIntegrationCommand.setJointMaxima(i, maxPositionError, maxVelocity);
      }
   }

   private void updateInverseDynamicsCommandList()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(jointAccelerationIntegrationCommand);
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
   }

   public void holdPositionInChest()
   {
      boolean initializeToCurrent = stateMachine.getCurrentStateEnum() != HandControlMode.TASKSPACE;
      taskspaceControlState.holdPositionInChest(handControlFrame, initializeToCurrent);
      requestedState.set(taskspaceControlState.getStateEnum());
   }

   public void handleHandTrajectoryCommand(HandTrajectoryCommand command)
   {
      boolean success;

      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         boolean initializeToCurrent = stateMachine.getCurrentStateEnum() != HandControlMode.TASKSPACE;
         success = taskspaceControlState.handleHandTrajectoryCommand(command, handControlFrame, initializeToCurrent);
         if (success)
            requestedState.set(taskspaceControlState.getStateEnum());
         else
            PrintTools.warn(this, "Can't execute HandTrajectoryCommand! " + command.getRobotSide());
         return;
      case QUEUE:
         success = taskspaceControlState.queueHandTrajectoryCommand(command);
         if (!success)
         {
            holdPositionInJointspace();
            PrintTools.warn(this, "Can't execute HandTrajectoryCommand! " + command.getRobotSide());
         }
         return;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         return;
      }
   }

   public void holdPositionInJointspace()
   {
      jointspaceControlState.holdCurrentConfiguration();
      requestedState.set(jointspaceControlState.getStateEnum());
   }

   public void goHome(double trajectoryTime)
   {
      boolean initializeToCurrent = stateMachine.getCurrentStateEnum() != HandControlMode.JOINTSPACE;
      jointspaceControlState.goHome(trajectoryTime, initializeToCurrent);
      requestedState.set(jointspaceControlState.getStateEnum());
   }

   public void handleArmTrajectoryCommand(ArmTrajectoryCommand command)
   {
      boolean success;

      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         boolean initializeToCurrent = stateMachine.getCurrentStateEnum() != HandControlMode.JOINTSPACE;
         success = jointspaceControlState.handleArmTrajectoryCommand(command, initializeToCurrent);
         if (success)
            requestedState.set(jointspaceControlState.getStateEnum());
         else
            PrintTools.warn(this, "Can't execute ArmTrajectoryCommand! " + command.getRobotSide());
         return;
      case QUEUE:
         success = jointspaceControlState.queueArmTrajectoryCommand(command);
         if (!success)
         {
            holdPositionInJointspace();
            PrintTools.warn(this, "Can't execute ArmTrajectoryCommand! " + command.getRobotSide());
         }
         return;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         return;
      }
   }

   public void handleArmDesiredAccelerationsCommand(ArmDesiredAccelerationsCommand command)
   {
      if (!ControllerCommandValidationTools.checkArmDesiredAccelerationsCommand(controlledJoints, command))
         return;

      switch (command.getArmControlMode())
      {
      case IHMC_CONTROL_MODE:
         if (stateMachine.getCurrentStateEnum() == HandControlMode.USER_CONTROL_MODE)
            holdPositionInJointspace();
         return;
      case USER_CONTROL_MODE:
         boolean success = userControlModeState.handleArmDesiredAccelerationsMessage(command);
         if (success)
            requestedState.set(userControlModeState.getStateEnum());
         else
            PrintTools.warn(this, "Can't execute ArmDesiredAccelerationsCommand! " + command.getRobotSide());
         return;
      default:
         throw new RuntimeException("Unknown ArmControlMode: " + command.getArmControlMode());
      }
   }

   public void handleHandComplianceControlParametersCommand(HandComplianceControlParametersCommand command)
   {
      PrintTools.error(this, "HandComplianceControlParametersControllerCommand is not supported anymore. Needs to be reimplememted.");
   }

   public void requestLoadBearing()
   {
      if (isAtLeastOneJointPositionControlled)
      {
         PrintTools.error("Cannot do load bearing when the arms are position controlled.");
         return;
      }
      requestedState.set(loadBearingControlState.getStateEnum());
   }

   public boolean isLoadBearing()
   {
      if (isAtLeastOneJointPositionControlled)
         return false;
      return stateMachine.getCurrentStateEnum() == HandControlMode.LOAD_BEARING;
   }

   public void resetJointIntegrators()
   {
      for (OneDoFJoint joint : controlledJoints)
         joint.resetIntegrator();
   }

   public boolean isControllingPoseInWorld()
   {
      State<HandControlMode> currentState = stateMachine.getCurrentState();

      if (currentState == taskspaceControlState)
         return taskspaceControlState.getTrajectoryFrame() == worldFrame;

      return false;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (inverseDynamicsCommandList.isEmpty())
         return null;
      else
         return inverseDynamicsCommandList;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (HandControlMode mode : HandControlMode.values())
      {
         HandControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
}
