package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;

import java.util.List;
import java.util.function.LongSupplier;

public class BehaviorTreeSceneExecutor extends BehaviorTreeSceneState
{
   private final YOLOv8DetectionExecutor yolo;
   private final IsaacROSFoundationPoseCommunicatorMap foundationPose;

   private final List<BehaviorTreeSceneObjectExecutor> objects;

   public BehaviorTreeSceneExecutor(CRDTInfo crdtInfo,
                                    LongSupplier idSupplier,
                                    ROS2SyncedRobotModel syncedRobot,
                                    YOLOv8DetectionExecutor yolo,
                                    IsaacROSFoundationPoseCommunicatorMap foundationPose)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.yolo = yolo;
      this.foundationPose = foundationPose;

      objects = (List) super.objects;

   }

   public void update()
   {
      for (BehaviorTreeSceneObjectExecutor object : objects)
      {
         object.update();
      }

      for (String availableModelName : yolo.getAvailableModelNames())
      {

      }

      //      for (String availableModelName : yolo.getAvailableModelNames())
//      {
//
//      }

//      yolo.enableModel("door");

      //      yolo.
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(BehaviorTreeSceneObjectStateMessage message)
   {
      return new BehaviorTreeSceneObjectExecutor(message.getId(), crdtInfo, message.getTypeAsString());
   }
}
