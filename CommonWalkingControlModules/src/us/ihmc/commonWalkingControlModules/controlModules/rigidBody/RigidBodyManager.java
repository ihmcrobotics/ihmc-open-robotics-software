package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
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
import us.ihmc.tools.io.printing.PrintTools;

public class RigidBodyManager
{
   private final String bodyName;
   private final YoVariableRegistry registry;
   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final EnumYoVariable<RigidBodyControlMode> requestedState;
   private final BooleanYoVariable hasBeenInitialized;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;

   private final double[] initialJointPositions;
   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final FramePose initialPose = new FramePose();

   private final OneDoFJoint[] jointsOriginal;
   private final OneDoFJoint[] jointsAtDesiredPosition;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame rootFrame;

   public RigidBodyManager(RigidBody bodyToControl, RigidBody rootBody, HighLevelHumanoidControllerToolbox humanoidControllerToolbox,
         WalkingControllerParameters walkingControllerParameters, Map<BaseForControl, ReferenceFrame> controlFrameMap, ReferenceFrame rootFrame,
         YoVariableRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoVariableRegistry(namePrefix);
      DoubleYoVariable yoTime = humanoidControllerToolbox.getYoTime();
      bodyFrame = bodyToControl.getBodyFixedFrame();
      this.rootFrame = rootFrame;

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);
      requestedState = new EnumYoVariable<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);
      hasBeenInitialized = new BooleanYoVariable(namePrefix + "Initialized", registry);
      hasBeenInitialized.set(false);

      OneDoFJoint[] jointsToControl = ScrewTools.createOneDoFJointPath(rootBody, bodyToControl);
      jointsOriginal = jointsToControl;
      jointsAtDesiredPosition = ScrewTools.cloneOneDoFJointPath(rootBody, bodyToControl);
      initialJointPositions = new double[jointsOriginal.length];

      RigidBody elevator = humanoidControllerToolbox.getFullRobotModel().getElevator();
      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsOriginal, yoTime, registry);
      taskspaceControlState = new RigidBodyTaskspaceControlState(bodyName, bodyToControl, rootBody, elevator, controlFrameMap, rootFrame, yoTime, registry);
      userControlState = new RigidBodyUserControlState(bodyName, jointsToControl, yoTime, registry);

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
      if (hasBeenInitialized.getBooleanValue())
         return;
      hasBeenInitialized.set(true);

      holdInJointspace();
   }

   public void compute()
   {
      initialize();

      if (stateMachine.getCurrentState().abortState())
         holdInJointspace();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      // TODO hand specific:
      // - hold position if a joint becomes disabled
      // - handle joint acceleration integration commands
      // update command lists
   }

   public void holdOrientationInTaskspace()
   {
      computeDesiredOrientation(initialOrientation);
      initialOrientation.changeFrame(rootFrame);
      taskspaceControlState.holdOrientation(initialOrientation);
      requestState(taskspaceControlState.getStateEnum());
   }

   public void holdPoseInTaskspace()
   {
      computeDesiredPose(initialPose);
      initialPose.changeFrame(rootFrame);
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
      initialOrientation.changeFrame(command.getReferenceFrame());

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
      initialPose.changeFrame(command.getReferenceFrame());

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
         requestState(RigidBodyControlMode.JOINTSPACE);
      else
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
   }

   public void handleDesiredAccelerationsCommand(DesiredAccelerationCommand<?, ?> command)
   {
      // TODO: hand, head
      // check the control mode in the message
      // if it is USER forward command to the user controller and activate it if necessary

      if (userControlState.handleDesiredAccelerationsCommand(command))
      {
         requestState(RigidBodyControlMode.USER);
      }
      else
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
      }
   }

   public void handleGoHomeCommand(GoHomeCommand command)
   {
      // TODO: chest
      // check if rigid body is in command
      // if so go home from current desired
   }

   public void goToHomeFromCurrent(double trajectoryTime)
   {
      // TODO: chest
      // go to some default configuration starting from the current measured in taskspace
   }

   public void goHome()
   {
      // TODO: hand
      // go to some default configuration
      // hand: jointspace
   }

   public void requestLoadBearing()
   {
      // TODO: hand
      // check if load bearing is supported by rigid body
      // if it is switch state to load bearing
   }

   public boolean isLoadBearing()
   {
      // TODO: hand
      // return whether the rigid body is load bearing
      return false;
   }

   public void resetJointIntegrators()
   {
      // TODO: hand
      // reset joint integrators for all joints that are controlled by this module
   }

   public boolean isControllingPoseInFrame()
   {
      // TODO: hand
      // check if the taskspace controller is active and what frame it is controlling the rigid body in
      return false;
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData()
   {
      // TODO: hand
      // if there are position controlled joints in this module return their desireds here
      return null;
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
            desiredJointPositionsToPack[i] = jointspaceControlState.getJointDesiredPosition(jointsOriginal[i]);
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
            jointsAtDesiredPosition[i].setQ(jointspaceControlState.getJointDesiredPosition(jointsOriginal[i]));
            jointsAtDesiredPosition[i].setQd(jointspaceControlState.getJointDesiredVelocity(jointsOriginal[i]));
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

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
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
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }

}
