package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.PersistentDetection;

/**
 * Tracks a door panel via two YOLO persistent detections, one for the panel and one for the opening mechanism.
 */
public class BehaviorTreeSceneDoorPanelExecutor extends BehaviorTreeSceneObjectExecutor
{
   private PersistentDetection doorPanelPersistentDetection;

   public BehaviorTreeSceneDoorPanelExecutor(long id, CRDTInfo crdtInfo, ROS2SyncedRobotModel syncedRobot, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(id, crdtInfo, syncedRobot, definition);
   }

   public PersistentDetection getDoorPanelPersistentDetection()
   {
      return doorPanelPersistentDetection;
   }

   public void setDoorPanelPersistentDetection(PersistentDetection doorPanelPersistentDetection)
   {
      this.doorPanelPersistentDetection = doorPanelPersistentDetection;
   }
}
