package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

public class RigidBodyControlManager
{
   private final String bodyName;
   private final YoVariableRegistry registry;
   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final EnumYoVariable<RigidBodyControlMode> requestedState;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;

   private final double[] initialJointPositions;
   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final FramePose initialPose = new FramePose();

   private final OneDoFJoint[] jointsOriginal;
   private final OneDoFJoint[] jointsAtDesiredPosition;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame baseFrame;

   private final BooleanYoVariable allJointsEnabled;
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public RigidBodyControlManager(RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator, TObjectDoubleHashMap<String> homeConfiguration,
         List<String> positionControlledJointNames, Collection<ReferenceFrame> controlFrames, ReferenceFrame baseFrame, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoVariableRegistry(namePrefix);
      bodyFrame = bodyToControl.getBodyFixedFrame();
      this.baseFrame = baseFrame;

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);
      requestedState = new EnumYoVariable<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);

      OneDoFJoint[] jointsToControl = ScrewTools.createOneDoFJointPath(baseBody, bodyToControl);
      jointsOriginal = jointsToControl;
      jointsAtDesiredPosition = ScrewTools.cloneOneDoFJointPath(baseBody, bodyToControl);
      initialJointPositions = new double[jointsOriginal.length];

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsOriginal, homeConfiguration, yoTime, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState(bodyName, bodyToControl, baseBody, elevator, controlFrames, baseFrame, yoTime, registry);
      userControlState = new RigidBodyUserControlState(bodyName, jointsToControl, yoTime, registry);

      ArrayList<OneDoFJoint> positionControlledJoints = new ArrayList<>();
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         OneDoFJoint joint = jointsToControl[jointIdx];
         if (positionControlledJointNames.contains(joint.getName()))
            positionControlledJoints.add(joint);
      }

      for (OneDoFJoint positionControlledJoint : positionControlledJoints)
      {
         lowLevelOneDoFJointDesiredDataHolder.registerJointWithEmptyData(positionControlledJoint);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(positionControlledJoint, LowLevelJointControlMode.POSITION_CONTROL);
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(positionControlledJoint);

         // TODO: add individual integration settings for joints
      }

      allJointsEnabled = new BooleanYoVariable(namePrefix + "AllJointsEnabled", registry);
      allJointsEnabled.set(true);

      setupStateMachine();
      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      List<RigidBodyControlState> states = new ArrayList<>();
      states.add(jointspaceControlState);
      states.add(taskspaceControlState);
      states.add(userControlState);

      for (RigidBodyControlState fromState : states)
      {
         for (RigidBodyControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
         stateMachine.addState(fromState);
      }
   }

   public void setWeights(TObjectDoubleHashMap<String> jointspaceWeights, Vector3D taskspaceAngularWeight, Vector3D taskspaceLinearWeight,
         TObjectDoubleHashMap<String> userModeWeights)
   {
      jointspaceControlState.setWeights(jointspaceWeights);
      taskspaceControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
      userControlState.setWeights(userModeWeights);
   }

   public void setGains(Map<String, YoPIDGains> jointspaceGains, YoOrientationPIDGainsInterface taskspaceOrientationGains,
         YoPositionPIDGainsInterface taskspacePositionGains)
   {
      jointspaceControlState.setGains(jointspaceGains);
      taskspaceControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
   }

   public void initialize()
   {
      holdInJointspace();
   }

   public void compute()
   {
      checkForDisabledJoints();

      if (stateMachine.getCurrentState().abortState())
         holdInJointspace();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   public void holdOrientationInTaskspace()
   {
      computeDesiredOrientation(initialOrientation);
      initialOrientation.changeFrame(baseFrame);
      taskspaceControlState.holdOrientation(initialOrientation);
      requestState(taskspaceControlState.getStateEnum());
   }

   public void holdPoseInTaskspace()
   {
      computeDesiredPose(initialPose);
      initialPose.changeFrame(baseFrame);
      taskspaceControlState.holdPose(initialPose);
      requestState(taskspaceControlState.getStateEnum());
   }

   public void holdInJointspace()
   {
      jointspaceControlState.holdCurrent();
      requestState(jointspaceControlState.getStateEnum());
   }

   private void holdOrientation()
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.TASKSPACE)
         holdOrientationInTaskspace();
      else
         holdInJointspace();
   }

   private void holdPose()
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.TASKSPACE)
         holdPoseInTaskspace();
      else
         holdInJointspace();
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      stateMachine.getCurrentState().handleStopAllTrajectoryCommand(command);
   }

   public void handleTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> command)
   {
      computeDesiredOrientation(initialOrientation);
      initialOrientation.changeFrame(command.getExpressedInFrame());

      if (taskspaceControlState.handleOrientationTrajectoryCommand(command, initialOrientation))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid orientation trajectory command.");
         holdOrientation();
      }
   }

   public void handleTaskspaceTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command)
   {
      computeDesiredPose(initialPose);
      initialPose.changeFrame(command.getExpressedInFrame());

      if (taskspaceControlState.handlePoseTrajectoryCommand(command, initialPose))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid pose trajectory command.");
         holdPose();
      }
   }

   public void handleJointspaceTrajectoryCommand(JointspaceTrajectoryCommand<?, ?> command)
   {
      computeDesiredJointPositions(initialJointPositions);

      if (jointspaceControlState.handleTrajectoryCommand(command, initialJointPositions))
      {
         requestState(jointspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
         holdInJointspace();
      }
   }

   public void handleDesiredAccelerationsCommand(DesiredAccelerationCommand<?, ?> command)
   {
      if (userControlState.handleDesiredAccelerationsCommand(command))
      {
         requestState(userControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
         holdInJointspace();
      }
   }

   public void goToHomeFromCurrent(double trajectoryTime)
   {
      jointspaceControlState.goHomeFromCurrent(trajectoryTime);
      requestState(jointspaceControlState.getStateEnum());
   }

   public void goHome(double trajectoryTime)
   {
      computeDesiredJointPositions(initialJointPositions);
      jointspaceControlState.goHome(trajectoryTime, initialJointPositions);
      requestState(jointspaceControlState.getStateEnum());
   }

   public void requestLoadBearing()
   {
      // TODO: check if load bearing is supported by rigid body
      // if it is switch state to load bearing
   }

   public boolean isLoadBearing()
   {
      // TODO: return whether the rigid body is load bearing
      return false;
   }

   public void resetJointIntegrators()
   {
      // TODO: reset joint integrators for all joints that are controlled by this module
   }

   public boolean isControllingPoseInFrame(ReferenceFrame referenceFrame)
   {
      // TODO: check if the taskspace controller is active and whether it is controlling the body in the given frame
      return false;
   }

   private void computeDesiredPose(FramePose desiredPoseToPack)
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.TASKSPACE)
      {
         taskspaceControlState.getDesiredPose(desiredPoseToPack);
      }
      else
      {
         updateJointsAtDesiredPosition();
         ReferenceFrame desiredEndEffectorFrame = jointsAtDesiredPosition[jointsAtDesiredPosition.length - 1].getSuccessor().getBodyFixedFrame();
         desiredPoseToPack.setToZero(desiredEndEffectorFrame);
      }

      if (desiredPoseToPack.containsNaN())
         desiredPoseToPack.setToZero(bodyFrame);
   }

   private void computeDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.TASKSPACE)
      {
         taskspaceControlState.getDesiredOrientation(desiredOrientationToPack);
      }
      else
      {
         updateJointsAtDesiredPosition();
         ReferenceFrame desiredEndEffectorFrame = jointsAtDesiredPosition[jointsAtDesiredPosition.length - 1].getSuccessor().getBodyFixedFrame();
         desiredOrientationToPack.setToZero(desiredEndEffectorFrame);
      }

      if (desiredOrientationToPack.containsNaN())
         desiredOrientationToPack.setToZero(bodyFrame);
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateEnum() == RigidBodyControlMode.JOINTSPACE)
      {
         for (int i = 0; i < jointsOriginal.length; i++)
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
            jointsAtDesiredPosition[i].setQ(jointsOriginal[i].getQ());
            jointsAtDesiredPosition[i].setQd(jointsOriginal[i].getQd());
         }
      }

      jointsAtDesiredPosition[0].updateFramesRecursively();
   }

   private void requestState(RigidBodyControlMode state)
   {
      if (stateMachine.getCurrentStateEnum() != state)
         requestedState.set(state);
   }

   private void checkForDisabledJoints()
   {
      boolean isAtLeastOneJointDisabled = checkIfAtLeastOneJointIsDisabled();

      if (isAtLeastOneJointDisabled && allJointsEnabled.getBooleanValue())
      {
         holdInJointspace();
         allJointsEnabled.set(false);
      }
      else if (!isAtLeastOneJointDisabled)
      {
         allJointsEnabled.set(true);
      }
   }

   private boolean checkIfAtLeastOneJointIsDisabled()
   {
      for (int jointIdx = 0; jointIdx < jointsOriginal.length; jointIdx++)
      {
         if (!jointsOriginal[jointIdx].isEnabled())
            return true;
      }
      return false;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
      inverseDynamicsCommandList.addCommand(jointAccelerationIntegrationCommand);
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

}
