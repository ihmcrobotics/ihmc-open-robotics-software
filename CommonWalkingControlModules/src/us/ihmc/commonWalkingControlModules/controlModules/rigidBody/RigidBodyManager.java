package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

public class RigidBodyManager
{
   private final YoVariableRegistry registry;
   private final GenericStateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final EnumYoVariable<RigidBodyControlMode> requestedState;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;

   public RigidBodyManager(RigidBody body, HighLevelHumanoidControllerToolbox humanoidControllerToolbox, YoVariableRegistry parentRegistry)
   {
      String namePrefix = body.getName() + "Manager";
      registry = new YoVariableRegistry(namePrefix);
      DoubleYoVariable yoTime = humanoidControllerToolbox.getYoTime();

      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", RigidBodyControlMode.class, yoTime, registry);
      requestedState = new EnumYoVariable<>("chestRequestedControlMode", registry, RigidBodyControlMode.class, true);

      jointspaceControlState = new RigidBodyJointspaceControlState();
      taskspaceControlState = new RigidBodyTaskspaceControlState();
      userControlState = new RigidBodyUserControlState();

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

   public void setWeights()
   {
      // TODO: hand, chest, head
      // set QP weights for all the control states
   }

   public void initialize()
   {
      // TODO: hand, chest, head
      // do some hold current position thing in one of the states using the current measured
   }

   public void compute() /* called doControl for hand */
   {
      // TODO: hand, chest, head
      // execute the state machine current state
      // hand specific:
      // - hold position if a joint becomes disabled
      // - handle joint acceleration integration commands
      // update command lists
   }

   public void holdInTaskspace()
   {
      // TODO: hand, chest, head
      // hold the position in the base reference frame using current desired
   }

   public void holdInJointspace()
   {
      // TODO: hand
      // hold the position in the base reference frame using current desired
   }

   public void handleStoptAllTrajectoryCommand()
   {
      // TODO: chest
      // stop executing trajectories and freeze the desireds
   }

   public void handleTaskspaceTrajectoryCommand()
   {
      // TODO: hand, chest, head
      // forward command to the taskspace controller and activate it if necessary
   }

   public void handleJointspaceTrajectoryCommand()
   {
      // TODO: hand, chest, head
      // forward command to the jointspace controller and activate it if necessary
   }

   public void handleDesiredAccelerationsCommand()
   {
      // TODO: hand, head
      // check the control mode in the message
      // if it is USER forward command to the user controller and activate it if necessary
   }

   public void handleGoHomeCommand()
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
