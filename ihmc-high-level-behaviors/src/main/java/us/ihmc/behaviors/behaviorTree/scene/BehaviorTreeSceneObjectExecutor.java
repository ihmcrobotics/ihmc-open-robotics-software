package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;

import java.time.Instant;

/**
 * Also known as the "Tracker"
 */
public class BehaviorTreeSceneObjectExecutor extends BehaviorTreeSceneObjectState
{
   protected final ROS2SyncedRobotModel syncedRobot;
   private PersistentDetection persistentDetection;
   private final PersistentDetectionMessageTool persistentDetectionMessageTool = new PersistentDetectionMessageTool();

   public BehaviorTreeSceneObjectExecutor(long id, CRDTInfo crdtInfo, ROS2SyncedRobotModel syncedRobot, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(id, crdtInfo, definition);

      this.syncedRobot = syncedRobot;
   }

   public void update()
   {
      if (!frozen.getValue() && persistentDetection != null && persistentDetection.isStable())
      {
         Vector3DBasics translation = persistentDetection.getFilteredTransform().getTranslation();

         Orientation3DReadOnly orientation;
         if (persistentDetection.getMostRecentDetection() instanceof YOLOv8InstantDetection)
            orientation = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getChestFrame).getOrientation();
         else
            orientation = persistentDetection.getFilteredTransform().getRotation();

         if (!(transform.getValueReadOnly().getRotation().geometricallyEquals(orientation, 1e-5)
             && transform.getValueReadOnly().getTranslation().epsilonEquals(translation, 1e-5)))
            transform.getValueAndModify().set(orientation, translation);
         referenceFrame.update();
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

   public boolean isStable()
   {
      return persistentDetection != null && persistentDetection.isStable();
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
