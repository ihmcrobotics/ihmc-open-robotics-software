package us.ihmc.behaviors.behaviorTree.action.actions;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionDefinition.SuccessCriteria;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.tools.Timer;

public class AbilityHandActionExecutor extends ActionNodeExecutor<AbilityHandActionState, AbilityHandActionDefinition>
{
   private final Timer timer = new Timer();

   public AbilityHandActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      AbilityHandState handState = abilityHandComms.get(definition.getSide()).getLatestState();
      if (handState != null)
         for (int i = 0; i < 6; i++)
            state.getCurrentFingerPositions().setValue(i, handState.getActuatorPositions()[i]);
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Executing Ability Hand action for side: {} with grip {}", definition.getSide(), definition.getGrip());

      AbilityHandActionComms comms = abilityHandComms.get(definition.getSide());
      if (comms.isConnected())
      {
         float ultimateTimeout = definition.getUltimateTimeout();
         state.setNominalExecutionDuration(ultimateTimeout);
         state.setPositionDistanceToGoalTolerance(Math.toRadians(10.0));

         state.getCommandedJointTrajectories().clear(6);
         for (int i = 0; i < 6; i++)
            state.getCommandedJointTrajectories().addTrajectoryPoint(i, Math.toRadians(comms.getLatestState().getActuatorPositions()[i]), 0.0);

         AbilityHandCommand command = comms.getCommand();
         command.setControlMode(definition.getControlMode().toByte());
         if (definition.getControlMode() == AbilityHandControlMode.GRIP)
         {
            AbilityHandGrip grip = definition.getGrip();
            command.setGrip(grip.toByte());

            double stageLength = ultimateTimeout / grip.getNumberOfStages();
            for (int s = 0; s < grip.getNumberOfStages(); s++)
            {
               for (int i = 0; i < grip.getFingersInStage(s); i++)
               {
                  int finger = grip.getStageFingerIndex(s, i);
                  double position = grip.getStageFingerPosition(s, i);
                  state.getCommandedJointTrajectories().addTrajectoryPoint(finger, Math.toRadians(position), (s + 1) * stageLength);
               }
            }
         }
         else
         {
            for (int i = 0; i < 6; i++)
            {
               float position = definition.getGoalPositions().getValueReadOnly(i);
               command.getGoalPositions()[i] = position;
               state.getCommandedJointTrajectories().addTrajectoryPoint(i, Math.toRadians(position), ultimateTimeout);
            }
         }
         for (int i = 0; i < 6; i++)
            command.getGoalVelocities()[i] = definition.getGoalVelocities().getValueReadOnly(i);
         comms.publishCommand();
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
      double elapsedTime = timer.getElapsedTime();
      state.setElapsedExecutionTime(elapsedTime);

      if (!timer.isRunning(definition.getUltimateTimeout()))
      {
         if (definition.getSuccessCriteria() == SuccessCriteria.WAIT_ONLY)
            state.getLogger().info("Waited %.1f s.".formatted(elapsedTime));
         else
            state.getLogger().error("Timed out after %.1f s.".formatted(elapsedTime));
         state.setFailed(definition.getSuccessCriteria() != SuccessCriteria.WAIT_ONLY);
         state.setIsExecuting(false);
      }

      AbilityHandActionComms comms = abilityHandComms.get(definition.getSide());
      AbilityHandState handState = comms.getLatestState();
      if (handState != null)
      {
         // TODO: Look at SCS YoVariable data to inform better strategies
         //   Possibly add "current checkpoints", succeed once each joint breaks some current level
         //   Possible wait some time after success to allow for grasp completion

         for (int i = 0; i < 6; i++)
         {
            state.getCurrentFingerPositions().setValue(i, handState.getActuatorPositions()[i]);
            state.getDesiredFingerPositions().setValue(i, handState.getGoalPositions()[i]);
            state.getCurrentJointAngles().setValue(i, Math.toRadians(handState.getActuatorPositions()[i]));
         }

         switch (definition.getSuccessCriteria())
         {
            case CHECK_EACH_JOINT_POSITION ->
            {
               boolean success = true;
               for (int i = 0; i < 6; i++)
               {
                  double error = Math.abs(definition.getGoalPositions().getValueReadOnly(i) - handState.getActuatorPositions()[i]);
                  success &= error <= definition.getEachJointPositionTolerance();
               }
               if (success)
               {
                  state.getLogger().info("Success: All fingers within tolerance of %.2f %s.".formatted(definition.getEachJointPositionTolerance(),
                                                                                                       EuclidCoreMissingTools.DEGREE_SYMBOL));
                  state.setIsExecuting(false);
               }
            }
            case CHECK_CUMULATIVE_JOINT_MOVEMENT ->
            {
               double cumulativeMovement = 0.0;
               for (int i = 0; i < 6; i++)
               {
                  double movement = Math.abs(handState.getActuatorPositions()[i]
                                             - Math.toDegrees(state.getCommandedJointTrajectories().getFirstValueReadOnly(i).getPosition()));
                  cumulativeMovement += movement;
               }
               if (cumulativeMovement >= definition.getSufficientCumulativeJointMovement())
               {
                  state.getLogger().info("Success: Moved cumulative %.2f %s.".formatted(cumulativeMovement, EuclidCoreMissingTools.DEGREE_SYMBOL));
                  state.setIsExecuting(false);
               }
            }
         }

         if (definition.getEnableWiggleOnFailure())
         {
            if (!timer.isRunning(definition.getTimeToWiggle()))
            {
               AbilityHandCommand command = comms.getCommand();
               command.setControlMode(AbilityHandControlMode.POSITION.toByte());
               for (int i = 0; i < 6; i++)
               {
                  float position = definition.getGoalPositions().getValueReadOnly(i);
                  float wiggleAmplitude = 7.0f;
                  float wiggleFrequency = 2.0f;
                  command.getGoalPositions()[i] = position + wiggleAmplitude * (float) Math.sin(wiggleFrequency * 2 * Math.PI * timer.getElapsedTime());
                  command.getGoalVelocities()[i] = definition.getGoalVelocities().getValueReadOnly(i);
               }
               comms.publishCommand();
            }
         }
      }
      else
      {
         state.getLogger().error("Failed to read hand state.");
         state.setFailed(true);
         state.setIsExecuting(false);
      }
   }
}
