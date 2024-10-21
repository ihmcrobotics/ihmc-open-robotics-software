package us.ihmc.behaviors.buildingExploration.old;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicBoolean;

class BuildingExplorationBehaviorWalkThroughDoorState implements State
{
   private final BehaviorHelper helper;
   final AtomicBoolean isDone = new AtomicBoolean();
   final AtomicBoolean receivedOperatorConfirmation = new AtomicBoolean();
   final AtomicBoolean hasStartedBehavior = new AtomicBoolean();
   private final ROS2SyncedRobotModel syncedRobot;

   public BuildingExplorationBehaviorWalkThroughDoorState(BehaviorHelper helper)
   {
      this.helper = helper;
      String robotName = helper.getRobotModel().getSimpleRobotName();

      syncedRobot = helper.newSyncedRobot();

      helper.subscribeToBehaviorStatusViaCallback(status ->
                                                  {
                                                     if (status == CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING)
                                                     {
                                                        isDone.set(true);
                                                     }
                                                  });
   }

   @Override
   public void onEntry()
   {
      BuildingExplorationBehaviorOld.pitchChestToSeeDoor(syncedRobot, helper);

      receivedOperatorConfirmation.set(false);
      hasStartedBehavior.set(false);
      isDone.set(false);

      LogTools.info("Entering " + getClass().getSimpleName());
   }

   private void startBehavior()
   {
      helper.publishBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR);
   }

   @Override
   public void doAction(double timeInState)
   {
      if (receivedOperatorConfirmation.get() && !hasStartedBehavior.get())
      {
         startBehavior();
         hasStartedBehavior.set(true);

         LogTools.info("Sent packet to start " + HumanoidBehaviorType.WALK_THROUGH_DOOR);
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);

      LogTools.info("Exiting " + getClass().getSimpleName());
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isDone.get();
   }

   public void proceedWithDoorBehavior()
   {
      receivedOperatorConfirmation.set(true);
   }
}
