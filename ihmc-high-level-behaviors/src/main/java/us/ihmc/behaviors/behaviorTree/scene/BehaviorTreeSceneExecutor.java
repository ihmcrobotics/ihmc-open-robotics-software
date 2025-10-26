package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collections;

public class BehaviorTreeSceneExecutor extends BehaviorTreeSceneState
{
   private final YOLOv8DetectionExecutor yolo;

   public BehaviorTreeSceneExecutor(ROS2SyncedRobotModel syncedRobot, YOLOv8DetectionExecutor yolo)
   {
      this.yolo = yolo;

      referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
      referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(side))
            referenceFrameLibrary.addAll(Collections.singleton(syncedRobot.getReferenceFrames().getHandZUpFrame(side)));
      }


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
