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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbstractLoadBearingCommand;
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
   public static final double INITIAL_GO_HOME_TIME = 2.0;

   private final String bodyName;
   private final YoVariableRegistry registry;
   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final EnumYoVariable<RigidBodyControlMode> requestedState;

   private final RigidBodyPositionControlHelper positionControlHelper;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;
   private final RigidBodyLoadBearingControlState loadBearingControlState;

   private final double[] initialJointPositions;
   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final FramePose initialPose = new FramePose();
   private final ReferenceFrame controlFrame;

   private final OneDoFJoint[] jointsOriginal;

   private final BooleanYoVariable allJointsEnabled;
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final BooleanYoVariable hasBeenInitialized;

   public RigidBodyControlManager(RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator, TObjectDoubleHashMap<String> homeConfiguration,
         List<String> positionControlledJointNames, Map<String, JointAccelerationIntegrationSettings> integrationSettings,
         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoVariableRegistry(namePrefix);
      this.controlFrame = controlFrame;

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);
      requestedState = new EnumYoVariable<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);
      hasBeenInitialized = new BooleanYoVariable(namePrefix + "HasBeenInitialized", registry);

      OneDoFJoint[] jointsToControl = ScrewTools.createOneDoFJointPath(baseBody, bodyToControl);
      jointsOriginal = jointsToControl;
      initialJointPositions = new double[jointsOriginal.length];

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsOriginal, homeConfiguration, yoTime, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState(bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame, baseFrame, yoTime, registry);
      userControlState = new RigidBodyUserControlState(bodyName, jointsToControl, yoTime, registry);
      loadBearingControlState = new RigidBodyLoadBearingControlState(bodyToControl, elevator, yoTime, registry);

      positionControlHelper = new RigidBodyPositionControlHelper(bodyName, jointsToControl, positionControlledJointNames, integrationSettings, registry);

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
      if (!hasBeenInitialized.getBooleanValue())
      {
         goToHomeFromCurrent(INITIAL_GO_HOME_TIME);
         hasBeenInitialized.set(true);
      }
   }

   public void compute()
   {
      initialize();

      checkForDisabledJoints();

      if (stateMachine.getCurrentState().abortState())
         holdInJointspace();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      positionControlHelper.update();
   }

   public void holdInJointspace()
   {
      jointspaceControlState.holdCurrent();
      requestState(jointspaceControlState.getStateEnum());
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (command.isStopAllTrajectory())
         holdInJointspace();
   }

   public void handleTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> command)
   {
      initialOrientation.setToZero(controlFrame);
      initialOrientation.changeFrame(command.getReferenceFrame());

      if (taskspaceControlState.handleOrientationTrajectoryCommand(command, initialOrientation))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid orientation trajectory command.");
         holdInJointspace();
      }
   }

   public void handleTaskspaceTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command)
   {
      initialPose.setToZero(controlFrame);
      initialPose.changeFrame(command.getReferenceFrame());

      if (taskspaceControlState.handlePoseTrajectoryCommand(command, initialPose))
      {
         requestState(taskspaceControlState.getStateEnum());
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid pose trajectory command.");
         holdInJointspace();
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
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid desired accelerations command.");
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

   public void handleLoadBearingCommand(AbstractLoadBearingCommand<?, ?> command)
   {
      if (positionControlHelper.hasPositionControlledJoints())
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " can not go to load bearing since some joints are position controlled.");
         return;
      }

      if (!command.getLoad())
      {
         holdInJointspace();
         return;
      }

      loadBearingControlState.setCoefficientOfFriction(command.getCoefficientOfFriction());
      loadBearingControlState.setContactNormalInWorldFrame(command.getContactNormalInWorldFrame());
      loadBearingControlState.setContactPoint(command.getContactPointInBodyFrame());
      requestState(loadBearingControlState.getStateEnum());
   }

   public boolean isLoadBearing()
   {
      return stateMachine.getCurrentStateEnum() == loadBearingControlState.getStateEnum();
   }

   public void resetJointIntegrators()
   {
      for (int jointIdx = 0; jointIdx < jointsOriginal.length; jointIdx++)
         jointsOriginal[jointIdx].resetIntegrator();
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateEnum() == jointspaceControlState.getStateEnum())
      {
         for (int i = 0; i < jointsOriginal.length; i++)
            desiredJointPositionsToPack[i] = jointspaceControlState.getJointDesiredPosition(i);
      }
      else
      {
         for (int i = 0; i < jointsOriginal.length; i++)
            desiredJointPositionsToPack[i] = jointsOriginal[i].getQ();
      }
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
      inverseDynamicsCommandList.addCommand(positionControlHelper.getJointAccelerationIntegrationCommand());
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
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }

}
