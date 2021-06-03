package us.ihmc.behaviors.door;

import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNodeBasics;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools.NAN_POSE;
import static us.ihmc.behaviors.door.DoorBehaviorAPI.*;

public class DoorBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Door", DoorBehavior::new, DoorBehaviorAPI.create());
   private final BehaviorHelper helper;
   private RemoteSyncedRobotModel syncedRobot;
   private final Notification doorConfirmed;
   private final AtomicReference<Boolean> reviewEnabled;
   private boolean firstTick = true;
   private boolean behaviorStarted = false;
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final Pose3D doorPose = new Pose3D(NAN_POSE);
   private double distanceToDoor;

   public DoorBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
      doorConfirmed = helper.subscribeTypelessViaNotification(DoorConfirmed);
      reviewEnabled = helper.subscribeViaReference(ReviewEnabled, true);
      helper.subscribeToDoorLocationViaCallback(doorLocationPacket -> doorPose.set(doorLocationPacket.getDoorTransformToWorld()));
   }

   public void setSyncedRobot(RemoteSyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
   }

   public boolean isDone()
   {
      return status.get() == CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING;
   }

   @Override
   public void clock()
   {
      distanceToDoor = doorPose.getPosition().distance(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getPosition());
      helper.publish(DistanceToDoor, distanceToDoor);
      super.clock();
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      if (firstTick)
      {
         firstTick = false;
         BuildingExplorationBehaviorTools.pitchChestToSeeDoor(syncedRobot, helper);

         if (!reviewEnabled.get())
            startBehavior();
      }
      if (!behaviorStarted && doorConfirmed.poll())
      {
         startBehavior();
      }

      return BehaviorTreeNodeStatus.RUNNING;
   }

   private void startBehavior()
   {
      behaviorStarted = true;
      helper.publishBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR);
      LogTools.info("Sending {}", HumanoidBehaviorType.WALK_THROUGH_DOOR.name());
   }

   @Override
   public void reset()
   {
      firstTick = true;
      behaviorStarted = false;
      helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);
      doorConfirmed.poll();
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   public Pose3DReadOnly getDoorPose()
   {
      return doorPose;
   }

   public double getDistanceToDoor()
   {
      return distanceToDoor;
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
