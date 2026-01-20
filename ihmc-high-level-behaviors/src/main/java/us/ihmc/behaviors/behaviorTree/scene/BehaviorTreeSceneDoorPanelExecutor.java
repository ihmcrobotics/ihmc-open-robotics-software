package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.perception.detections.PersistentDetection;

import java.time.Instant;

/**
 * Tracks a door panel via two YOLO persistent detections, one for the panel and one for the opening mechanism.
 */
public class BehaviorTreeSceneDoorPanelExecutor extends BehaviorTreeSceneObjectExecutor
{
   private PersistentDetection panelDetection;
   private final RotationMatrix orientation = new RotationMatrix();
   private final Vector3D mechanismToPanel = new Vector3D();
   private final PersistentDetectionMessageTool persistentDetectionMessageTool = new PersistentDetectionMessageTool();

   public BehaviorTreeSceneDoorPanelExecutor(long id, CRDTInfo crdtInfo, ROS2SyncedRobotModel syncedRobot, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(id, crdtInfo, syncedRobot, definition);
   }

   @Override
   public void update()
   {
      PersistentDetection mechanismDetection = getPersistentDetection();

      if (mechanismDetection != null && panelDetection != null)
      {
         if (mechanismDetection.isStable() && panelDetection.isStable())
         {
            Vector3DBasics mechanismPosition = mechanismDetection.getFilteredTransform().getTranslation();

            // Set orientation to Z-up, then yaw so Y axis points from mechanism to panel
            mechanismToPanel.set(panelDetection.getFilteredTransform().getTranslation());
            mechanismToPanel.sub(mechanismPosition);
            double yaw = Math.atan2(mechanismToPanel.getY(), mechanismToPanel.getX());
            orientation.setToYawOrientation(yaw);

            if (!(transform.getValueReadOnly().getRotation().geometricallyEquals(orientation, 1e-5)
               && transform.getValueReadOnly().getTranslation().epsilonEquals(mechanismPosition, 1e-5)))
               transform.getValueAndModify().set(orientation, mechanismPosition);
         }
      }
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      if (panelDetection != null)
         persistentDetectionMessageTool.toMessage(syncedRobot, Instant.now(), panelDetection, message.getDoorPanelDetection());
   }


   public PersistentDetection getDoorPanelPersistentDetection()
   {
      return panelDetection;
   }

   public void setDoorPanelPersistentDetection(PersistentDetection panelDetection)
   {
      this.panelDetection = panelDetection;
   }
}
