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
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbstractLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class RigidBodyControlManager
{
   public static final double INITIAL_GO_HOME_TIME = 2.0;

   private final String bodyName;
   private final YoVariableRegistry registry;
   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final YoEnum<RigidBodyControlMode> requestedState;
   private final EnumParameter<RigidBodyControlMode> defaultControlMode;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;
   private final RigidBodyLoadBearingControlState loadBearingControlState;

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform();
   private final double[] initialJointPositions;
   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D homePose;

   private final OneDoFJoint[] jointsToControl;

   private final YoBoolean allJointsEnabled;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final YoBoolean stateSwitched;

   public RigidBodyControlManager(RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator, TObjectDoubleHashMap<String> homeConfiguration,
                                  Pose3D homePose, Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame,
                                  ContactablePlaneBody contactableBody, RigidBodyControlMode defaultControlMode, YoDouble yoTime,
                                  YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoVariableRegistry(namePrefix);

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);
      stateSwitched = new YoBoolean(namePrefix + "StateSwitched", registry);

      jointsToControl = ScrewTools.createOneDoFJointPath(baseBody, bodyToControl);

      initialJointPositions = new double[jointsToControl.length];

      RigidBodyJointControlHelper jointControlHelper = new RigidBodyJointControlHelper(bodyName, jointsToControl, parentRegistry);

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsToControl, homeConfiguration, yoTime, jointControlHelper, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState("", bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame, baseFrame, yoTime,
                                                                 jointControlHelper, graphicsListRegistry, registry);
      userControlState = new RigidBodyUserControlState(bodyName, jointsToControl, yoTime, registry);

      if (contactableBody != null)
         loadBearingControlState = new RigidBodyLoadBearingControlState(bodyToControl, contactableBody, elevator, yoTime, jointControlHelper,
                                                                        graphicsListRegistry, registry);
      else
         loadBearingControlState = null;

      if (homePose != null)
         this.homePose = new FramePose3D(baseFrame, homePose);
      else
         this.homePose = null;

      defaultControlMode = defaultControlMode == null ? RigidBodyControlMode.JOINTSPACE : defaultControlMode;
      checkDefaultControlMode(defaultControlMode, this.homePose, bodyName);
      String description = "WARNING: only " + RigidBodyControlMode.JOINTSPACE + " or " + RigidBodyControlMode.TASKSPACE + " possible!";
      this.defaultControlMode = new EnumParameter<>(namePrefix + "DefaultControlMode", description, registry, RigidBodyControlMode.class, false,
            defaultControlMode);
      this.defaultControlMode.addParameterChangedListener(parameter -> checkDefaultControlMode(this.defaultControlMode.getValue(), this.homePose, bodyName));

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

   public void setWeights(Map<String, DoubleProvider> jointspaceWeights, Vector3DReadOnly taskspaceAngularWeight, Vector3DReadOnly taskspaceLinearWeight,
                          Map<String, DoubleProvider> userModeWeights)
   {
      jointspaceControlState.setDefaultWeights(jointspaceWeights);
      taskspaceControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
      userControlState.setWeights(userModeWeights);
      if (loadBearingControlState != null)
         loadBearingControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
   }

   public void setGains(Map<String, PIDGainsReadOnly> jointspaceGains, PID3DGainsReadOnly taskspaceOrientationGains,
                        PID3DGainsReadOnly taskspacePositionGains)
   {
      jointspaceControlState.setGains(jointspaceGains);
      taskspaceControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
      if (loadBearingControlState != null)
         loadBearingControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
   }

   private static void checkDefaultControlMode(RigidBodyControlMode defaultControlMode, FramePose3D homePose, String bodyName)
   {
      if (defaultControlMode == null)
      {
         throw new RuntimeException("Default control mode can not be null for body " + bodyName + ".");
      }

      if (defaultControlMode == RigidBodyControlMode.TASKSPACE && homePose == null)
      {
         throw new RuntimeException("Need to define home pose if default control mode for body " + bodyName + " is set to TASKSPACE.");
      }

      if (defaultControlMode != RigidBodyControlMode.TASKSPACE && defaultControlMode != RigidBodyControlMode.JOINTSPACE)
      {
         throw new RuntimeException("Only JOINTSPACE or TASKSPACE control modes are allowed as default modes for body " + bodyName + ".");
      }
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
      switch (defaultControlMode.getValue())
      {
      case JOINTSPACE:
         holdInJointspace();
         break;
      case TASKSPACE:
         holdInTaskspace();
         break;
      default:
         throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
      }
   }

   public void goToHomeFromCurrent(double trajectoryTime)
   {
      switch (defaultControlMode.getValue())
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
         throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
      }
   }

   public void goHome(double trajectoryTime)
   {
      switch (defaultControlMode.getValue())
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
         throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
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

   private void computeDesiredPose(FramePose3D poseToPack)
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

   public String getControllerBodyName()
   {
      return bodyName;
   }
   
   public OneDoFJoint[] getControlledJoints()
   {
      return jointsToControl;
   }
}
