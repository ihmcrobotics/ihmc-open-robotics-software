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
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationSettings;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbstractLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

public class RigidBodyControlManager
{
   public static final double INITIAL_GO_HOME_TIME = 2.0;

   private final String bodyName;
   private final YoVariableRegistry registry;
   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final YoEnum<RigidBodyControlMode> requestedState;
   private final YoEnum<RigidBodyControlMode> defaultControlMode;

   private final RigidBodyPositionControlHelper positionControlHelper;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;
   private final RigidBodyLoadBearingControlState loadBearingControlState;

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform();
   private final double[] initialJointPositions;
   private final FramePose initialPose = new FramePose();
   private final FramePose homePose;

   private final OneDoFJoint[] jointsToControl;

   private final YoBoolean allJointsEnabled;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final YoBoolean stateSwitched;

   public RigidBodyControlManager(RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator, TObjectDoubleHashMap<String> homeConfiguration,
         Pose3D homePose, List<String> positionControlledJointNames, Map<String, JointAccelerationIntegrationSettings> integrationSettings,
         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame, ContactablePlaneBody contactableBody,
         YoDouble yoTime, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoVariableRegistry(namePrefix);

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);
      stateSwitched = new YoBoolean(namePrefix + "StateSwitched", registry);

      defaultControlMode = new YoEnum<>(namePrefix + "DefaultControlMode", registry, RigidBodyControlMode.class, true);
      defaultControlMode.set(RigidBodyControlMode.JOINTSPACE);

      jointsToControl = ScrewTools.createOneDoFJointPath(baseBody, bodyToControl);

      initialJointPositions = new double[jointsToControl.length];

      positionControlHelper = new RigidBodyPositionControlHelper(bodyName, jointsToControl, positionControlledJointNames, integrationSettings, registry);
      RigidBodyJointControlHelper jointControlHelper = new RigidBodyJointControlHelper(bodyName, jointsToControl, parentRegistry);

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsToControl, homeConfiguration, yoTime, jointControlHelper, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState("", bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame, baseFrame, yoTime,
                                                                 jointControlHelper, graphicsListRegistry, registry);
      userControlState = new RigidBodyUserControlState(bodyName, jointsToControl, yoTime, registry);

      if (!positionControlHelper.hasPositionControlledJoints() && contactableBody != null)
         loadBearingControlState = new RigidBodyLoadBearingControlState(bodyToControl, contactableBody, elevator, yoTime, jointControlHelper,
                                                                        graphicsListRegistry, registry);
      else
         loadBearingControlState = null;

      if (homePose != null)
         this.homePose = new FramePose(baseFrame, homePose);
      else
         this.homePose = null;

      allJointsEnabled = new YoBoolean(namePrefix + "AllJointsEnabled", registry);
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
      if (loadBearingControlState != null)
         states.add(loadBearingControlState);

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
      jointspaceControlState.setDefaultWeights(jointspaceWeights);
      taskspaceControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
      userControlState.setWeights(userModeWeights);
      if (loadBearingControlState != null)
         loadBearingControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
   }

   public void setGains(Map<String, YoPIDGains> jointspaceGains, YoOrientationPIDGainsInterface taskspaceOrientationGains,
         YoPositionPIDGainsInterface taskspacePositionGains)
   {
      jointspaceControlState.setGains(jointspaceGains);
      taskspaceControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
      if (loadBearingControlState != null)
         loadBearingControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
   }

   public void setDefaultControlMode(RigidBodyControlMode defaultControlMode)
   {
      if (defaultControlMode == RigidBodyControlMode.TASKSPACE && homePose == null)
      {
         throw new RuntimeException("Need to define home pose if default control mode for body " + bodyName + " is set to TASKSPACE.");
      }

      if (defaultControlMode != RigidBodyControlMode.TASKSPACE && defaultControlMode != RigidBodyControlMode.JOINTSPACE)
      {
         throw new RuntimeException("Only JOINTSPACE or TASKSPACE control modes are allowed as default modes.");
      }

      this.defaultControlMode.set(defaultControlMode);
   }

   public void initialize()
   {
      goToHomeFromCurrent(INITIAL_GO_HOME_TIME);
   }

   public void compute()
   {
      checkForDisabledJoints();

      if (stateMachine.getCurrentState().abortState())
         hold();

      stateSwitched.set(stateMachine.checkTransitionConditions());

      if (stateSwitched.getBooleanValue())
      {
         stateMachine.getPreviousState().clear();
      }

      stateMachine.doAction();

      positionControlHelper.update();
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (command.isStopAllTrajectory())
         hold();
   }

   public void handleTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> command)
   {
      if (command.useCustomControlFrame())
      {
         command.getControlFramePose(controlFrameTransform);
         taskspaceControlState.setControlFramePose(controlFrameTransform);
      }
      else
      {
         taskspaceControlState.setDefaultControlFrame();
      }

      computeDesiredPose(initialPose);

      if (taskspaceControlState.handleOrientationTrajectoryCommand(command, initialPose))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid orientation trajectory command.");
         taskspaceControlState.clear();
         hold();
      }
   }

   public void handleTaskspaceTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command)
   {
      if (command.useCustomControlFrame())
      {
         command.getControlFramePose(controlFrameTransform);
         taskspaceControlState.setControlFramePose(controlFrameTransform);
      }
      else
      {
         taskspaceControlState.setDefaultControlFrame();
      }

      computeDesiredPose(initialPose);

      if (taskspaceControlState.handlePoseTrajectoryCommand(command, initialPose))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid pose trajectory command.");
         taskspaceControlState.clear();
         hold();
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
         hold();
      }
   }

   public void handleHybridTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> taskspaceCommand, JointspaceTrajectoryCommand<?, ?> jointSpaceCommand)
   {
      if (taskspaceCommand.useCustomControlFrame())
      {
         taskspaceCommand.getControlFramePose(controlFrameTransform);
         taskspaceControlState.setControlFramePose(controlFrameTransform);
      }
      else
      {
         taskspaceControlState.setDefaultControlFrame();
      }

      computeDesiredJointPositions(initialJointPositions);
      computeDesiredPose(initialPose);

      if (taskspaceControlState.handleHybridPoseTrajectoryCommand(taskspaceCommand, initialPose, jointSpaceCommand, initialJointPositions))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid hybrid SE3 trajectory command.");
         hold();
      }
   }

   public void handleHybridTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> taskspaceCommand, JointspaceTrajectoryCommand<?, ?> jointspaceCommand)
   {
      throw new RuntimeException("Should not send these messages anymore. Switch to SE3 message with selection matrix.");
//      if (taskspaceCommand.useCustomControlFrame())
//      {
//         taskspaceCommand.getControlFramePose(controlFrameTransform);
//         taskspaceControlState.setControlFramePose(controlFrameTransform);
//      }
//      else
//      {
//         taskspaceControlState.setDefaultControlFrame();
//      }
//
//      computeDesiredJointPositions(initialJointPositions);
//      computeDesiredPose(initialPose);
//
//      if (hybridControlState.handleTrajectoryCommand(taskspaceCommand, jointspaceCommand, initialJointPositions, initialPose))
//      {
//         requestState(hybridControlState.getStateEnum());
//      }
//      else
//      {
//         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid hybrid SO3 trajectory command.");
//         hold();
//      }
   }

   public void handleDesiredAccelerationsCommand(DesiredAccelerationCommand<?, ?> command)
   {
      if (userControlState.handleDesiredAccelerationsCommand(command))
      {
         requestState(userControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid desired accelerations command.");
         hold();
      }
   }

   public void holdInJointspace()
   {
      jointspaceControlState.holdCurrent();
      requestState(jointspaceControlState.getStateEnum());
   }

   public void holdInTaskspace()
   {
      taskspaceControlState.holdCurrent();
      requestState(taskspaceControlState.getStateEnum());
   }

   public void hold()
   {
      switch (defaultControlMode.getEnumValue())
      {
      case JOINTSPACE:
         holdInJointspace();
         break;
      case TASKSPACE:
         holdInTaskspace();
         break;
      default:
         PrintTools.warn("Default control mode " + defaultControlMode.getEnumValue() + " is not an implemented option.");
         defaultControlMode.set(RigidBodyControlMode.JOINTSPACE);
         hold();
         break;
      }
   }

   public void goToHomeFromCurrent(double trajectoryTime)
   {
      switch (defaultControlMode.getEnumValue())
      {
      case JOINTSPACE:
         jointspaceControlState.goHomeFromCurrent(trajectoryTime);
         requestState(jointspaceControlState.getStateEnum());
         break;
      case TASKSPACE:
         taskspaceControlState.goToPoseFromCurrent(homePose, trajectoryTime);
         requestState(taskspaceControlState.getStateEnum());
         break;
      default:
         PrintTools.warn("Default control mode " + defaultControlMode.getEnumValue() + " is not an implemented option.");
         defaultControlMode.set(RigidBodyControlMode.JOINTSPACE);
         goToHomeFromCurrent(trajectoryTime);
         break;
      }
   }

   public void goHome(double trajectoryTime)
   {
      switch (defaultControlMode.getEnumValue())
      {
      case JOINTSPACE:
         computeDesiredJointPositions(initialJointPositions);
         jointspaceControlState.goHome(trajectoryTime, initialJointPositions);
         requestState(jointspaceControlState.getStateEnum());
         break;
      case TASKSPACE:
         taskspaceControlState.setDefaultControlFrame();
         computeDesiredPose(initialPose);
         taskspaceControlState.goToPose(homePose, initialPose, trajectoryTime);
         requestState(taskspaceControlState.getStateEnum());
         break;
      default:
         PrintTools.warn("Default control mode " + defaultControlMode.getEnumValue() + " is not an implemented option.");
         defaultControlMode.set(RigidBodyControlMode.JOINTSPACE);
         goHome(trajectoryTime);
         break;
      }
   }

   public void handleLoadBearingCommand(AbstractLoadBearingCommand<?, ?> command, JointspaceTrajectoryCommand<?, ?> jointspaceCommand)
   {
      if (loadBearingControlState == null)
      {
         PrintTools.info(getClass().getSimpleName() + " for " + bodyName + " can not go to load bearing.");
         return;
      }

      if (!command.getLoad())
      {
         hold();
         return;
      }

      if (jointspaceCommand != null)
      {
         computeDesiredJointPositions(initialJointPositions);
         if (!loadBearingControlState.handleJointTrajectoryCommand(jointspaceCommand, initialJointPositions))
            return;
      }

      if (loadBearingControlState.handleLoadbearingCommand(command))
      {
         requestState(loadBearingControlState.getStateEnum());
      }
   }

   public boolean isLoadBearing()
   {
      if (loadBearingControlState == null)
         return false;

      return stateMachine.getCurrentStateEnum() == loadBearingControlState.getStateEnum();
   }

   public void resetJointIntegrators()
   {
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
         jointsToControl[jointIdx].resetIntegrator();
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateEnum() == jointspaceControlState.getStateEnum())
      {
         for (int i = 0; i < jointsToControl.length; i++)
            desiredJointPositionsToPack[i] = jointspaceControlState.getJointDesiredPosition(i);
      }
      else
      {
         for (int i = 0; i < jointsToControl.length; i++)
            desiredJointPositionsToPack[i] = jointsToControl[i].getQ();
      }
   }

   private void computeDesiredPose(FramePose poseToPack)
   {
      if (stateMachine.getCurrentStateEnum() == taskspaceControlState.getStateEnum())
      {
         taskspaceControlState.getDesiredPose(poseToPack);
      }
      else
      {
         poseToPack.setToZero(taskspaceControlState.getControlFrame());
      }
   }

   private void requestState(RigidBodyControlMode state)
   {
      if (stateMachine.getCurrentStateEnum() != state)
         requestedState.set(state);
   }

   public RigidBodyControlMode getActiveControlMode()
   {
      return stateMachine.getCurrentStateEnum();
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
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         if (!jointsToControl[jointIdx].isEnabled())
            return true;
      }
      return false;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
      inverseDynamicsCommandList.addCommand(positionControlHelper.getJointAccelerationIntegrationCommand());

      if (stateSwitched.getBooleanValue())
      {
         RigidBodyControlState previousState = stateMachine.getPreviousState();
         InverseDynamicsCommand<?> transitionOutOfStateCommand = previousState.getTransitionOutOfStateCommand();
         inverseDynamicsCommandList.addCommand(transitionOutOfStateCommand);
      }

      return inverseDynamicsCommandList;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData()
   {
      return positionControlHelper.getLowLevelOneDoFJointDesiredDataHolder();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RigidBodyControlMode mode : RigidBodyControlMode.values())
      {
         RigidBodyControlState state = stateMachine.getState(mode);
         if (state != null && state.createFeedbackControlTemplate() != null)
            ret.addCommand(state.createFeedbackControlTemplate());
      }
      return ret;
   }

}
