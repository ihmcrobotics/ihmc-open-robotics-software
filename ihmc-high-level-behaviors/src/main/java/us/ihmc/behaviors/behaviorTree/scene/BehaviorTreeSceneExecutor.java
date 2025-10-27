package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;

import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeSceneExecutor extends BehaviorTreeSceneState
{
   private final YOLOv8DetectionExecutor yolo;
   private final IsaacROSFoundationPoseManager foundationPose;

   private final List<BehaviorTreeSceneObjectTracker> trackers = new ArrayList<>();

   public BehaviorTreeSceneExecutor(ROS2SyncedRobotModel syncedRobot, YOLOv8DetectionExecutor yolo, IsaacROSFoundationPoseManager foundationPose)
   {
      super(syncedRobot);

      this.yolo = yolo;
      this.foundationPose = foundationPose;



   }

   public void update()
   {
      for (String availableModelName : yolo.getAvailableModelNames())
      {

      }

      yolo.enableModel("door");

      //      yolo.
   }
}
