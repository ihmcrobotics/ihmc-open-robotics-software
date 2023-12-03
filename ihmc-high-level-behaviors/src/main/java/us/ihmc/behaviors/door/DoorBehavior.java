package us.ihmc.behaviors.door;

import perception_msgs.msg.dds.DetectedFiducialPacket;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.ResettingNode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.Timer;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A behavior tree layer on top of the door behavior that John wrote for Atlas.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class DoorBehavior extends ResettingNode implements Destroyable
{
   private BehaviorHelper helper;
   private ROS2SyncedRobotModel syncedRobot;
   private AtomicReference<Boolean> reviewEnabled;
   private boolean firstTick = true;
   private boolean doingBehavior = false;
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final AtomicReference<DetectedFiducialPacket> detectedFiducial = new AtomicReference<>();
   private final Pose3D doorPose = new Pose3D(BehaviorTools.createNaNPose());
   private double distanceToDoor = 0.0;
   private boolean isFacingDoor = false;
   private final Timer doorDetectedTimer = new Timer();

   public DoorBehavior(BehaviorHelper helper, ROS2SyncedRobotModel syncedRobot)
   {
      create(helper);
      this.syncedRobot = syncedRobot;
   }

   public DoorBehavior(BehaviorHelper helper)
   {
      create(helper);
      syncedRobot = helper.newSyncedRobot();
   }

   private void create(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
      // FIXME: subscribe review enabled
      reviewEnabled = null;
      helper.subscribeToDoorLocationViaCallback(doorLocationPacket ->
      {
         doorDetectedTimer.reset();
         doorPose.set(doorLocationPacket.getDoorTransformToWorld());
         // publish detected door
      });
      helper.subscribeViaCallback(ROS2Tools::getBehaviorStatusTopic, status ->
      {
         CurrentBehaviorStatus currentBehaviorStatus = CurrentBehaviorStatus.fromByte(status.getCurrentBehaviorStatus());
         if (currentBehaviorStatus == CurrentBehaviorStatus.BEHAVIOR_FINISHED_SUCCESS
          || currentBehaviorStatus == CurrentBehaviorStatus.BEHAVIOR_FINISHED_FAILED)
         {
            doingBehavior = false;
         }
      });
      helper.getOrCreateControllerStatusTracker().addNotWalkingStateAnymoreCallback(() -> doingBehavior = false);
      //      helper.subscribeViaCallback(FiducialDetectorToolboxModule::getDetectedFiducialOutputTopic, detectedFiducialMessage ->
      //      {
      //         detectedFiducial.set(detectedFiducialMessage);
      //         helper.publish(DetectedDoorPose, new Pose3D(detectedFiducialMessage.getFiducialTransformToWorld()));
      //      });
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
      // FIXME: publish distance to door
      helper.publishToolboxState(FiducialDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);
      helper.publishToolboxState(ObjectDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);

      Vector3D doorTravelDirection = new Vector3D(Axis3D.Y);


//      doorPose.getOrientation().
//      helper.publish(IsFacingDoor, distanceToDoor);
      super.clock();
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      if (firstTick)
      {
         firstTick = false;

         if (!reviewEnabled.get())
         {
            startBehavior();
         }
      }
//      if (!behaviorStarted && doorConfirmed.poll())
//      {
//         startBehavior();
//      }

      return BehaviorTreeNodeStatus.RUNNING;
   }

   private void startBehavior()
   {
      doingBehavior = true;
      helper.publishBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR);
      LogTools.info("Sending {}", HumanoidBehaviorType.WALK_THROUGH_DOOR.name());
   }

   @Override
   public void reset()
   {
      LogTools.info("Would have stopped door behavior! (but didn't)");
      firstTick = true;
      doingBehavior = false;
//      helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);
   }

   public Pose3DReadOnly getDoorPose()
   {
      return doorPose;
   }

   public boolean isFacingDoor()
   {
      return isFacingDoor;
   }

   public boolean isDoingBehavior()
   {
      return doingBehavior;
   }

   public boolean hasSeenDoorRecently()
   {
      return doorDetectedTimer.isRunning(5.0);
   }

   public double getDistanceToDoor()
   {
      return distanceToDoor;
   }

   public String getName()
   {
      return "Door";
   }
}
