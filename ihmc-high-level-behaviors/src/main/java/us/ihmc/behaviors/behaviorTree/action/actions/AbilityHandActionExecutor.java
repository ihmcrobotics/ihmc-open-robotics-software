package us.ihmc.behaviors.behaviorTree.action.actions;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.tools.Timer;

import java.util.Arrays;

public class AbilityHandActionExecutor extends ActionNodeExecutor<AbilityHandActionState, AbilityHandActionDefinition>
{
   private String identifier = null;
   private final Timer timer = new Timer();

   public AbilityHandActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Executing Ability Hand action for side: {} with grip {}", definition.getSide(), definition.getGrip());

      AbilityHandCommand command = getCommand();
      AbilityHandState handState = readState();
      if (command != null && handState != null)
      {
         Arrays.fill(command.getGoalVelocities(), 30.0f);
         command.setControlMode(definition.getControlMode().toByte());
         if (definition.getControlMode() == ControlMode.GRIP)
         {
            command.setGrip(definition.getGrip().toByte());
         }
         else
         {
            for (int i = 0; i < 5; i++)
               command.getGoalPositions()[i] = definition.getGoalPositions().getValueReadOnly(i);
            command.getGoalPositions()[5] = -definition.getGoalPositions().getValueReadOnly(5);
         }
         for (int i = 0; i < 6; i++)
            command.getGoalVelocities()[i] = definition.getGoalVelocities().getValueReadOnly(i);
         abilityHandCommunication.publishCommand(identifier);
         timer.reset();
      }
      else
      {
         state.setFailed(true);
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      AbilityHandState handState = readState();
      if (handState != null && timer.isRunning(5.0)) // timeout 5 s
      {
         boolean moving = false;
         for (int i = 0; i < 6; i++)
            moving |= Math.abs(handState.getGoalVelocities()[i]) > 0.0f;
         if (moving)
            timer.reset();

//         double nominalExecutionDuration = 4.0;
//         state.getCommandedJointTrajectories().clear(6);
//         state.getCommandedJointTrajectories().addTr
         for (int i = 0; i < 6; i++)
         {
            state.getCurrentFingerPositions().setValue(i, handState.getActuatorPositions()[i]);
            state.getDesiredFingerPositions().setValue(i, handState.getGoalPositions()[i]);
         }

         if (!timer.isRunning(1.0)) // end after 1 s of stillness
            state.setIsExecuting(false);
      }
      else
      {
         state.setFailed(true);
         state.setIsExecuting(false);
      }
   }

   private String updateIdentifier()
   {
      identifier = HandInterface.getSimpleIdentifier(robotModel.getSimpleRobotName(), definition.getSide(), HandType.ABILITY_HAND);
      return identifier;
   }

   private AbilityHandCommand getCommand()
   {
      return abilityHandCommunication.getAvailableHands().contains(updateIdentifier()) ? abilityHandCommunication.getCommand(identifier) : null;
   }

   private AbilityHandState readState()
   {
      return abilityHandCommunication.getAvailableHands().contains(updateIdentifier()) ? abilityHandCommunication.readState(identifier) : null;
   }
}
