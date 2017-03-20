package us.ihmc.commonWalkingControlModules.controlModules.head;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

public class HeadOrientationManager
{
   private static final double defaultTrajectoryTime = 1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final GenericStateMachine<HeadControlMode, HeadControlState> stateMachine;
   private final EnumYoVariable<HeadControlMode> requestedState = new EnumYoVariable<>("headRequestedControlMode", registry, HeadControlMode.class, true);

   private final TaskspaceHeadControlState taskspaceHeadControlState;
   private final JointspaceHeadControlState jointspaceHeadControlState;
   private final HeadUserControlModeState headUserControlModeState;

   private final DoubleYoVariable yoTime;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasHeadOrientationManagerBeenInitialized", registry);

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   private final OneDoFJoint[] jointsOriginal;
   /**
    * These joints are cloned from {@link #jointsOriginal}.
    * Their positions and velocities are set to match the current desireds.
    * They are used as an easy way to figure out what would be the desired hand pose when the arm is controlled in jointspace.
    * They are updated by calling {@link #updateJointsAtDesiredPosition()}.
    */
   private final OneDoFJoint[] jointsAtDesiredPosition;
   private LowLevelOneDoFJointDesiredDataHolderReadOnly newJointDesiredData = null;

   private final ReferenceFrame headFrame;

   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final double[] initialJointPositions;

   public HeadOrientationManager(HighLevelHumanoidControllerToolbox controllerToolbox, HeadOrientationControllerParameters headOrientationControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      yoTime = controllerToolbox.getYoTime();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      stateMachine = new GenericStateMachine<>("headControlState", "headControlState" + "SwitchTime", HeadControlMode.class, yoTime, registry);

      RigidBody head = fullRobotModel.getHead();
      RigidBody chest = fullRobotModel.getChest();
      double[] homeYawPitchRoll = headOrientationControllerParameters.getInitialHeadYawPitchRoll();
      headFrame = head.getBodyFixedFrame();

      YoOrientationPIDGainsInterface taskspaceGains = headOrientationControllerParameters.createHeadOrientationControlGains(registry);
      YoPIDGains jointspaceGains = headOrientationControllerParameters.createHeadJointspaceControlGains(registry);
      jointsOriginal = ScrewTools.createOneDoFJointPath(chest, head);
      jointsAtDesiredPosition = ScrewTools.cloneOneDoFJointPath(chest, head);
      initialJointPositions = new double[jointsOriginal.length];

      List<HeadControlState> states = new ArrayList<>();
      taskspaceHeadControlState = new TaskspaceHeadControlState(chest, head, homeYawPitchRoll, taskspaceGains, registry);
      states.add(taskspaceHeadControlState);
      jointspaceHeadControlState = new JointspaceHeadControlState(jointsOriginal, jointspaceGains, registry);
      states.add(jointspaceHeadControlState);
      headUserControlModeState = new HeadUserControlModeState(jointsOriginal, yoTime, registry);
      states.add(headUserControlModeState);

      for (HeadControlState fromState : states)
      {
         for (HeadControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
         stateMachine.addState(fromState);
      }

      parentRegistry.addChild(registry);

      hasBeenInitialized.set(false);

      boolean neckPositionControlled = headOrientationControllerParameters.isNeckPositionControlled();

      if (neckPositionControlled)
      {
         jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
         lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

         lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointsOriginal);
         lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointsOriginal, LowLevelJointControlMode.POSITION_CONTROL);

         for (OneDoFJoint neckJoint : jointsOriginal)
            jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(neckJoint);
      }
      else
      {
         jointAccelerationIntegrationCommand = null;
         lowLevelOneDoFJointDesiredDataHolder = null;
      }
   }

   public void setWeights(double jointspace, double taskspace, double userMode)
   {
      jointspaceHeadControlState.setWeight(jointspace);
      taskspaceHeadControlState.setWeight(taskspace);
      headUserControlModeState.setWeight(userMode);
   }

   public void initialize()
   {
      if (hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);

      initialOrientation.setToZero(headFrame);
      taskspaceHeadControlState.handleGoHome(defaultTrajectoryTime, initialOrientation);
      requestedState.set(taskspaceHeadControlState.getStateEnum());
   }

   public void compute()
   {
      initialize();

      if (stateMachine.getCurrentStateEnum() == HeadControlMode.USER_CONTROL_MODE && headUserControlModeState.isAbortUserControlModeRequested())
         holdCurrentOrientation();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      updateInverseDynamicsCommandList();
   }

   public void holdCurrentOrientation()
   {
      taskspaceHeadControlState.holdCurrentOrientation();
      requestedState.set(taskspaceHeadControlState.getStateEnum());
   }

   public void handleHeadTrajectoryCommand(HeadTrajectoryCommand command)
   {
      computeDesiredOrientation(initialOrientation);
      taskspaceHeadControlState.handleHeadTrajectoryCommand(command, initialOrientation);
      requestedState.set(taskspaceHeadControlState.getStateEnum());
   }

   public void handleNeckTrajectoryCommand(NeckTrajectoryCommand command)
   {
      computeDesiredJointPositions(initialJointPositions);
      boolean success = jointspaceHeadControlState.handleNeckTrajectoryCommand(command, initialJointPositions);
      if (success)
         requestedState.set(jointspaceHeadControlState.getStateEnum());
   }

   public void handleNeckDesiredAccelerationsCommand(NeckDesiredAccelerationsCommand command)
   {
      headUserControlModeState.handleNeckDesiredAccelerationsMessage(command);
      requestedState.set(headUserControlModeState.getStateEnum());
   }

   private void updateInverseDynamicsCommandList()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(jointAccelerationIntegrationCommand);
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
   }

   private void computeDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      if (stateMachine.getCurrentStateEnum() == HeadControlMode.TASKSPACE)
      {
         taskspaceHeadControlState.getDesiredOrientation(initialOrientation);
      }
      else
      {
         updateJointsAtDesiredPosition();
         ReferenceFrame desiredEndEffectorFrame = jointsAtDesiredPosition[jointsAtDesiredPosition.length - 1].getSuccessor().getBodyFixedFrame();
         desiredOrientationToPack.setToZero(desiredEndEffectorFrame);
      }
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateEnum() == HeadControlMode.JOINTSPACE)
      {
         for (int i = 0; i < jointsOriginal.length; i++)
         {
            desiredJointPositionsToPack[i] = jointspaceHeadControlState.getJointDesiredPosition(jointsOriginal[i]);
         }
      }
      else
      {
         updateJointsAtDesiredPosition();
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            desiredJointPositionsToPack[i] = jointsAtDesiredPosition[i].getQ();
         }
      }
   }

   private void updateJointsAtDesiredPosition()
   {
      if (newJointDesiredData != null)
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            OneDoFJoint jointAtDesiredPosition = jointsAtDesiredPosition[i];
            double q = jointsOriginal[i].getQ();
            double qd = jointsOriginal[i].getQd();

            if (newJointDesiredData.hasDataForJoint(jointsOriginal[i]))
            {
               LowLevelJointDataReadOnly jointDesiredData = newJointDesiredData.getLowLevelJointData(jointsOriginal[i]);

               double qDesired = jointDesiredData.hasDesiredPosition() ? jointDesiredData.getDesiredPosition() : q;
               jointAtDesiredPosition.setQ(qDesired);

               double qdDesired = jointDesiredData.hasDesiredVelocity() ? jointDesiredData.getDesiredVelocity() : qd;
               jointAtDesiredPosition.setQd(qdDesired);
            }
            else
            {
               jointAtDesiredPosition.setQ(q);
               jointAtDesiredPosition.setQd(qd);
            }
         }

         newJointDesiredData = null;
      }
      else if (stateMachine.getCurrentStateEnum() == HeadControlMode.JOINTSPACE)
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            jointsAtDesiredPosition[i].setQ(jointspaceHeadControlState.getJointDesiredPosition(jointsOriginal[i]));
            jointsAtDesiredPosition[i].setQd(jointspaceHeadControlState.getJointDesiredVelocity(jointsOriginal[i]));
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

   /**
    * In a best effort of having continuity in desireds between states, the low-level data can be used to update the {@link HandControlModule} with
    * the most recent desired joint positions and velocities.
    * @param lowLevelOneDoFJointDesiredDataHolder Data that will be used to update the arm desired configuration. Only a read-only access is needed.
    */
   public void submitNewNeckJointDesiredConfiguration(LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      newJointDesiredData = lowLevelOneDoFJointDesiredDataHolder;
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
      for (HeadControlMode mode : HeadControlMode.values())
      {
         HeadControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
}
