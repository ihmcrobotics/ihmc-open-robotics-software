package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.Collections;

public class BehaviorTreeScene
{
   private final ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
   private final YOLOv8DetectionExecutor yolo;

   public BehaviorTreeScene(ROS2SyncedRobotModel syncedRobot, YOLOv8DetectionExecutor yolo)
   {
      this.yolo = yolo;

      referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
      referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
   }

   public void update()
   {
      for (String availableModelName : yolo.getAvailableModelNames())
      {

      }

      yolo.enableModel("door");

//      yolo.
   }

   public ReferenceFrameLibrary getReferenceFrameLibrary()
   {
      return referenceFrameLibrary;
   }
}
