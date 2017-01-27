package us.ihmc.commonWalkingControlModules.controlModules.chest;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
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

public class ChestOrientationManagerNew implements ChestOrientationManagerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GenericStateMachine<ChestControlMode, ChestControlState> stateMachine;
   private final EnumYoVariable<ChestControlMode> requestedState = new EnumYoVariable<>("headRequestedControlMode", registry, ChestControlMode.class, true);

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable(getClass().getSimpleName() + "Initialized", registry);

   private final TaskspaceChestControlState taskspaceChestControlState;
   private final JointspaceChestControlState jointspaceChestControlState;

   private final ReferenceFrame chestFrame;
   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final double[] initialJointPositions;

   private final OneDoFJoint[] jointsOriginal;
   private final OneDoFJoint[] jointsAtDesiredPosition;

   private LowLevelOneDoFJointDesiredDataHolderReadOnly newJointDesiredData = null;

   public ChestOrientationManagerNew(HighLevelHumanoidControllerToolbox humanoidControllerToolbox, YoOrientationPIDGainsInterface gains, Vector3d angularWeight, YoVariableRegistry parentRegistry)
   {
      String className = getClass().getSimpleName();
      DoubleYoVariable yoTime = humanoidControllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(className, className + "SwitchTime", ChestControlMode.class, yoTime, registry);

      RigidBody chest = humanoidControllerToolbox.getFullRobotModel().getChest();
      RigidBody pelvis = humanoidControllerToolbox.getFullRobotModel().getPelvis();
      chestFrame = chest.getBodyFixedFrame();
      jointsOriginal = ScrewTools.createOneDoFJointPath(pelvis, chest);
      jointsAtDesiredPosition = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      initialJointPositions = new double[jointsOriginal.length];

      taskspaceChestControlState = new TaskspaceChestControlState(humanoidControllerToolbox, gains, angularWeight, parentRegistry);
      jointspaceChestControlState = new JointspaceChestControlState();

      setupStateMachine();
      parentRegistry.addChild(registry);
      hasBeenInitialized.set(false);
   }

   private void setupStateMachine()
   {
      List<ChestControlState> states = new ArrayList<>();
      states.add(taskspaceChestControlState);
      states.add(jointspaceChestControlState);

      for (ChestControlState fromState : states)
      {
         for (ChestControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
         stateMachine.addState(fromState);
      }

      stateMachine.setCurrentState(ChestControlMode.TASK_SPACE);
   }

   public void setWeights(double jointspace, Vector3d taskspace)
   {
      jointspaceChestControlState.setWeight(jointspace);
      taskspaceChestControlState.setWeights(taskspace);
   }

   @Override
   public void initialize()
   {
      if (hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);
      holdCurrentOrientation();
   }

   @Override
   public void holdCurrentOrientation()
   {
      computeDesiredOrientation(initialOrientation);
      taskspaceChestControlState.holdOrientation(initialOrientation);
      requestState(taskspaceChestControlState.getStateEnum());
   }

   @Override
   public void compute()
   {
      initialize();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   @Override
   public void handleChestTrajectoryCommand(ChestTrajectoryCommand command)
   {
      computeDesiredOrientation(initialOrientation);
      taskspaceChestControlState.handleChestTrajectoryCommand(command, initialOrientation);
      requestState(taskspaceChestControlState.getStateEnum());
   }

   @Override
   public void handleSpineTrajectoryCommand(SpineTrajectoryCommand command)
   {
      computeDesiredJointPositions(initialJointPositions);
      boolean success = jointspaceChestControlState.handleNeckTrajectoryCommand(command, initialJointPositions);
      if (!success)
         holdCurrentOrientation();
      requestState(jointspaceChestControlState.getStateEnum());
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (command.isStopAllTrajectory())
         holdCurrentOrientation();
   }

   @Override
   public void handleGoHomeCommand(GoHomeCommand command)
   {
      if (command.getRequest(BodyPart.CHEST))
         goToHomeFromCurrentDesired(command.getTrajectoryTime());
   }

   @Override
   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      computeDesiredOrientation(initialOrientation);

      taskspaceChestControlState.goToHome(trajectoryTime, initialOrientation);
      requestState(taskspaceChestControlState.getStateEnum());
   }

   @Override
   public void goToHomeFromCurrent(double trajectoryTime)
   {
      initialOrientation.setToZero(chestFrame);

      taskspaceChestControlState.goToHome(trajectoryTime, initialOrientation);
      requestState(taskspaceChestControlState.getStateEnum());
   }

   private void requestState(ChestControlMode state)
   {
      if (stateMachine.getCurrentStateEnum() != state)
         requestedState.set(state);
   }

   private void computeDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      if (stateMachine.getCurrentStateEnum() == ChestControlMode.TASK_SPACE)
      {
         taskspaceChestControlState.getDesiredOrientation(initialOrientation);
      }
      else
      {
         updateJointsAtDesiredPosition();
         ReferenceFrame desiredEndEffectorFrame = jointsAtDesiredPosition[jointsAtDesiredPosition.length - 1].getSuccessor().getBodyFixedFrame();
         desiredOrientationToPack.setToZero(desiredEndEffectorFrame);
      }

      if (initialOrientation.containsNaN())
         initialOrientation.setToZero(chestFrame);
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateEnum() == ChestControlMode.JOINT_SPACE)
      {
         for (int i = 0; i < jointsOriginal.length; i++)
         {
            desiredJointPositionsToPack[i] = jointspaceChestControlState.getJointDesiredPosition(jointsOriginal[i]);
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
      else if (stateMachine.getCurrentStateEnum() == ChestControlMode.JOINT_SPACE)
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            jointsAtDesiredPosition[i].setQ(jointspaceChestControlState.getJointDesiredPosition(jointsOriginal[i]));
            jointsAtDesiredPosition[i].setQd(jointspaceChestControlState.getJointDesiredVelocity(jointsOriginal[i]));
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
   @Override
   public void submitNewSpineJointDesiredConfiguration(LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      newJointDesiredData = lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   @Override
   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (ChestControlMode mode : ChestControlMode.values())
      {
         ChestControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
}
