package us.ihmc.behaviors.door;

import controller_msgs.msg.dds.DetectedFiducialPacket;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
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
   private ROS2SyncedRobotModel syncedRobot;
   private final Notification doorConfirmed;
   private final AtomicReference<Boolean> reviewEnabled;
   private boolean firstTick = true;
   private boolean behaviorStarted = false;
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final AtomicReference<DetectedFiducialPacket> detectedFiducial = new AtomicReference<>();
   private final Pose3D doorPose = new Pose3D(NAN_POSE);
   private double distanceToDoor = 0.0;
   private boolean isFacingDoor = false;

   public DoorBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
      doorConfirmed = helper.subscribeTypelessViaNotification(DoorConfirmed);
      reviewEnabled = helper.subscribeViaReference(ReviewEnabled, true);
      helper.subscribeToDoorLocationViaCallback(doorLocationPacket ->
      {
         doorPose.set(doorLocationPacket.getDoorTransformToWorld());
         helper.publish(DetectedDoorPose, MutablePair.of(DoorType.fromByte(doorLocationPacket.getDetectedDoorType()), new Pose3D(doorPose)));
      });
//      helper.subscribeViaCallback(FiducialDetectorToolboxModule::getDetectedFiducialOutputTopic, detectedFiducialMessage ->
//      {
//         detectedFiducial.set(detectedFiducialMessage);
//         helper.publish(DetectedDoorPose, new Pose3D(detectedFiducialMessage.getFiducialTransformToWorld()));
//      });
   }

   public void setSyncedRobot(ROS2SyncedRobotModel syncedRobot)
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
      FramePose3DReadOnly robotPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
      distanceToDoor = doorPose.getPosition().distance(robotPose.getPosition());
      helper.publish(DistanceToDoor, distanceToDoor);
      helper.publishToolboxState(FiducialDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);
      helper.publishToolboxState(ObjectDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);

      Vector3D doorTravelDirection = new Vector3D(Axis3D.Y);


//      doorPose.getOrientation().
//      helper.publish(IsFacingDoor, distanceToDoor);
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
//      if (!behaviorStarted && doorConfirmed.poll())
//      {
//         startBehavior();
//      }

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

   public boolean isFacingDoor()
   {
      return isFacingDoor;
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
