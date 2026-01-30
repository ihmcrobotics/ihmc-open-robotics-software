package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters;

public class BehaviorTreeSceneDoorFrameExecutor extends BehaviorTreeSceneObjectExecutor
{
   private BehaviorTreeSceneDoorPanelExecutor doorPanel;

   private final Pose3D hingePostPose;
   private final Pose3D latchPostPose;
   private double panelWidth;

   public BehaviorTreeSceneDoorFrameExecutor(long id, CRDTInfo crdtInfo, ROS2SyncedRobotModel syncedRobot, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(id, crdtInfo, syncedRobot, definition);

      hingePostPose = new Pose3D();
      hingePostPose.setToNaN();

      latchPostPose = new Pose3D();
      latchPostPose.setToNaN();

      panelWidth = DoorModelParameters.DOOR_PANEL_WIDTH;
   }

   @Override
   public void update()
   {
      super.update();

      if (doorPanel != null && doorPanel.isStable())
      {
         RigidBodyTransform transformToWorld = transform.getValueAndModify();
         transformToWorld.set(doorPanel.getTransformToWorld());
         transformToWorld.appendTranslation(panelWidth + 0.05, 0.0, 0.0);

         hingePostPose.set(transformToWorld);
         latchPostPose.set(hingePostPose);
         latchPostPose.prependTranslation(panelWidth + 0.2, 0.0, 0.0);
      }
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      message.getHingePostPose().set(hingePostPose);
      message.getLatchPostPose().set(latchPostPose);
   }

   public void setPanelWidth(double panelWidth)
   {
   this.panelWidth = panelWidth;
   }

   public void setDoorPanel(BehaviorTreeSceneDoorPanelExecutor doorPanel)
   {
      this.doorPanel = doorPanel;
   }
}
