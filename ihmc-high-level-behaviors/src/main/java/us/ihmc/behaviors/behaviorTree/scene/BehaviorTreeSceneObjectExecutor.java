package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

import java.time.Instant;

/**
 * Also known as the "Tracker"
 */
public class BehaviorTreeSceneObjectExecutor extends BehaviorTreeSceneObjectState
{
   private final ROS2SyncedRobotModel syncedRobot;

   private PersistentDetection persistentDetection;

   private final PersistentDetectionMessageTool persistentDetectionMessageTool = new PersistentDetectionMessageTool();

   public BehaviorTreeSceneObjectExecutor(long id, CRDTInfo crdtInfo, ROS2SyncedRobotModel syncedRobot, IsaacROSFoundationPoseObject objectType)
   {
      super(id, crdtInfo, objectType);

      this.syncedRobot = syncedRobot;
   }

   public void update()
   {
      if (persistentDetection != null)
      {
         if (persistentDetection.isStable())
         {
            transform.setValue(persistentDetection.getFilteredTransform(), 1e-5);
         }
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      if (persistentDetection != null)
         persistentDetectionMessageTool.toMessage(syncedRobot, Instant.now(), persistentDetection, message.getPersistentDetection());
   }

   public PersistentDetection getPersistentDetection()
   {
      return persistentDetection;
   }

   public void setPersistentDetection(PersistentDetection persistentDetection)
   {
      this.persistentDetection = persistentDetection;
   }
}
