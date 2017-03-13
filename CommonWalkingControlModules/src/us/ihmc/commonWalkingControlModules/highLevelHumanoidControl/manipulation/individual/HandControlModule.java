package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools.addRequestedStateTransition;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyLoadBearingControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyUserControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // JPratt. February 27, 2015: Added this since new Atlas was having trouble with network stuff.
   // It was sending 14,000 variables. This and others reduces it a bit when set to false.
   private static final boolean REGISTER_YOVARIABLES = true;

   private final YoVariableRegistry registry;

   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;

   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyLoadBearingControlState loadBearingControlState;
   private final RigidBodyUserControlState userControlModeState;

   private final EnumYoVariable<RigidBodyControlMode> requestedState;
   private final OneDoFJoint[] controlledJoints;

   private final Map<OneDoFJoint, BooleanYoVariable> areJointsEnabled;
   private final String name;
   private final RigidBody chest, hand;

   private final FullHumanoidRobotModel fullRobotModel;

   private final StateChangedListener<RigidBodyControlMode> stateChangedlistener;
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

   public HandControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, ArmControllerParameters armControlParameters,
         YoPIDGains jointspaceGains, YoSE3PIDGainsInterface taskspaceGains, YoVariableRegistry parentRegistry)
   {
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      hasHandPoseStatusBeenSent = new BooleanYoVariable(namePrefix + "HasHandPoseStatusBeenSent", registry);
      hasHandPoseStatusBeenSent.set(false);

      areAllArmJointEnabled = new BooleanYoVariable(namePrefix + "AreAllArmJointEnabled", registry);
      areAllArmJointEnabled.set(true);

      stateChangedlistener = new StateChangedListener<RigidBodyControlMode>()
      {
         @Override
         public void stateChanged(State<RigidBodyControlMode> oldState, State<RigidBodyControlMode> newState, double time)
         {
            hasHandPoseStatusBeenSent.set(false);
         }
      };

      fullRobotModel = controllerToolbox.getFullRobotModel();
      hand = fullRobotModel.getHand(robotSide);
      chest = fullRobotModel.getChest();

      chestFrame = chest.getBodyFixedFrame();
      handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

      controlledJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      jointsAtDesiredPosition = ScrewTools.cloneOneDoFJointPath(chest, hand);
      initialJointPositions = new double[controlledJoints.length];

      requestedState = new EnumYoVariable<RigidBodyControlMode>(name + "RequestedState", "", registry, RigidBodyControlMode.class, true);
      requestedState.set(null);

      areJointsEnabled = new LinkedHashMap<>();
      for (OneDoFJoint oneDoFJoint : controlledJoints)
      {
         areJointsEnabled.put(oneDoFJoint, new BooleanYoVariable(namePrefix + oneDoFJoint.getName() + "IsEnabled", registry));
      }

      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(name, name + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);

      RigidBody elevator = fullRobotModel.getElevator();
      String[] positionControlledJointNames = armControlParameters.getPositionControlledJointNames(robotSide);
      isAtLeastOneJointPositionControlled = positionControlledJointNames != null && positionControlledJointNames.length > 0;

      OneDoFJoint[] positionControlledJoints;
      if (isAtLeastOneJointPositionControlled)
         positionControlledJoints = ScrewTools.filterJoints(ScrewTools.findJointsWithNames(controlledJoints, positionControlledJointNames), OneDoFJoint.class);
      else
         positionControlledJoints = new OneDoFJoint[0];

      Map<OneDoFJoint, Double> homeConfiguration = armControlParameters.getDefaultArmJointPositions(fullRobotModel, robotSide);
      TObjectDoubleHashMap<String> homeConfigurationNew = new TObjectDoubleHashMap<>();
      for (OneDoFJoint joint : controlledJoints)
         homeConfigurationNew.put(joint.getName(), homeConfiguration.get(joint));
      String bodyName = hand.getName();

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, controlledJoints, homeConfigurationNew, yoTime, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState(bodyName, hand, chest, elevator, null, chestFrame, yoTime, registry);
      userControlModeState = new RigidBodyUserControlState(bodyName, controlledJoints, yoTime, registry);

      taskspaceControlState.setGains(taskspaceGains.getOrientationGains(), taskspaceGains.getPositionGains());
      jointspaceControlState.setGains(jointspaceGains);

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
         loadBearingControlState = new RigidBodyLoadBearingControlState(bodyName, yoTime, registry);
         jointAccelerationIntegrationCommand = null;
         lowLevelOneDoFJointDesiredDataHolder = null;
         accelerationIntegrationAlphaPosition = null;
         accelerationIntegrationAlphaVelocity = null;
         accelerationIntegrationMaxPositionError = null;
         accelerationIntegrationMaxVelocity = null;
      }

      parentRegistry.addChild(registry);

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      List<RigidBodyControlState> allStates = new ArrayList<>();
      allStates.add(jointspaceControlState);
      allStates.add(taskspaceControlState);
      allStates.add(userControlModeState);
      if (loadBearingControlState != null)
         allStates.add(loadBearingControlState);

      for (RigidBodyControlState fromState : allStates)
      {
         for (RigidBodyControlState toState : allStates)
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

      if (stateMachine.getCurrentState().abortState())
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
      computeDesiredPose(initialPose);
      taskspaceControlState.holdPose(initialPose);
      requestedState.set(taskspaceControlState.getStateEnum());
   }

   public void handleHandTrajectoryCommand(HandTrajectoryCommand command)
   {
      computeDesiredPose(initialPose);
      if (taskspaceControlState.handlePoseTrajectoryCommand(command, initialPose))
         requestState(taskspaceControlState.getStateEnum());
      else
         holdPositionInJointspace();
   }

   public void holdPositionInJointspace()
   {
      jointspaceControlState.holdCurrent();
      requestState(jointspaceControlState.getStateEnum());
   }

   public void goHome(double trajectoryTime)
   {
      jointspaceControlState.goHomeFromCurrent(trajectoryTime);
      requestState(jointspaceControlState.getStateEnum());
   }

   public void handleArmTrajectoryCommand(ArmTrajectoryCommand command)
   {
      computeDesiredJointPositions(initialJointPositions);
      if (jointspaceControlState.handleTrajectoryCommand(command, initialJointPositions))
         requestState(jointspaceControlState.getStateEnum());
      else
         holdPositionInJointspace();
   }

   public void handleArmDesiredAccelerationsCommand(ArmDesiredAccelerationsCommand command)
   {
      if (!ControllerCommandValidationTools.checkArmDesiredAccelerationsCommand(controlledJoints, command))
         return;

      boolean success = userControlModeState.handleDesiredAccelerationsCommand(command);
      if (success)
         requestState(userControlModeState.getStateEnum());
      else
         PrintTools.warn(this, "Can't execute ArmDesiredAccelerationsCommand! " + command.getRobotSide());
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
      return stateMachine.getCurrentStateEnum() == RigidBodyControlMode.LOAD_BEARING;
   }

   public void resetJointIntegrators()
   {
      for (OneDoFJoint joint : controlledJoints)
         joint.resetIntegrator();
   }

   public boolean isControllingPoseInWorld()
   {
      return true;
   }

   private void requestState(RigidBodyControlMode state)
   {
      if (stateMachine.getCurrentStateEnum() != state)
         requestedState.set(state);
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
      for (RigidBodyControlMode mode : RigidBodyControlMode.values())
      {
         RigidBodyControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }

   private final double[] initialJointPositions;
   private final FramePose initialPose = new FramePose();
   private final OneDoFJoint[] jointsAtDesiredPosition;

   private void computeDesiredPose(FramePose desiredPoseToPack)
   {
      desiredPoseToPack.setToZero(hand.getBodyFixedFrame());
      desiredPoseToPack.changeFrame(worldFrame);
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.JOINTSPACE)
      {
         for (int i = 0; i < controlledJoints.length; i++)
            desiredJointPositionsToPack[i] = jointspaceControlState.getJointDesiredPosition(i);
      }
      else
      {
         updateJointsAtDesiredPosition();
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
            desiredJointPositionsToPack[i] = jointsAtDesiredPosition[i].getQ();
      }
   }

   private void updateJointsAtDesiredPosition()
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.JOINTSPACE)
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            jointsAtDesiredPosition[i].setQ(jointspaceControlState.getJointDesiredPosition(i));
            jointsAtDesiredPosition[i].setQd(jointspaceControlState.getJointDesiredVelocity(i));
         }
      }
      else
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            jointsAtDesiredPosition[i].setQ(controlledJoints[i].getQ());
            jointsAtDesiredPosition[i].setQd(controlledJoints[i].getQd());
         }
      }

      jointsAtDesiredPosition[0].updateFramesRecursively();
   }

}
