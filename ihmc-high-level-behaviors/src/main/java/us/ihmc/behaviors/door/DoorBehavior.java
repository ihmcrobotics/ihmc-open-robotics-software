package us.ihmc.behaviors.door;

import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.tools.thread.ActivationReference;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.door.DoorBehaviorAPI.DoorConfirmed;

public class DoorBehavior extends ResettingNode implements BehaviorInterface
{
   private final BehaviorHelper helper;
   private final RemoteSyncedRobotModel syncedRobot;
   private boolean firstTick = true;
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final ActivationReference<Boolean> doorConfirmed;

   public DoorBehavior(BehaviorHelper helper, RemoteSyncedRobotModel syncedRobot)
   {
      this.helper = helper;
      this.syncedRobot = syncedRobot;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
      doorConfirmed = helper.subscribeViaActivationReference(DoorConfirmed);
   }

   public boolean isDone()
   {
      return status.get() == CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING;
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      if (firstTick)
      {
         firstTick = false;
         BuildingExplorationBehaviorTools.pitchChestToSeeDoor(syncedRobot, helper);
      }
      if (doorConfirmed.poll() && doorConfirmed.hasChanged())
      {
         helper.publishBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR);
      }

      return BehaviorTreeNodeStatus.RUNNING;
   }


   @Override
   public void reset()
   {
      firstTick = true;
      helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }
}
