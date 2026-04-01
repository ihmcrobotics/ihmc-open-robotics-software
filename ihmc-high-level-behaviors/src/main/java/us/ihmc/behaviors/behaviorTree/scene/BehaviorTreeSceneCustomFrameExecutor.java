package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class BehaviorTreeSceneCustomFrameExecutor extends BehaviorTreeSceneObjectExecutor
{
   private final BehaviorTreeSceneExecutor scene;
   private final Point3D frameAPosition = new Point3D();
   private final Point3D frameBPosition = new Point3D();
   private final Vector3D frameAToB = new Vector3D();

   public BehaviorTreeSceneCustomFrameExecutor(long id,
                                               CRDTInfo crdtInfo,
                                               ROS2SyncedRobotModel syncedRobot,
                                               BehaviorTreeSceneObjectDefinitionMessage definition,
                                               BehaviorTreeSceneExecutor scene)
   {
      super(id, crdtInfo, syncedRobot, definition);

      this.scene = scene;
   }

   @Override
   public void update()
   {
      if (frozen.getValue())
         return;

      ReferenceFrame frameAReference = scene.findFrameByName(getFrameA());
      ReferenceFrame frameBReference = scene.findFrameByName(getFrameB());
      if (frameAReference == null || frameBReference == null)
         return;

      frameAPosition.set(frameAReference.getTransformToRoot().getTranslation());
      frameBPosition.set(frameBReference.getTransformToRoot().getTranslation());

      frameAToB.sub(frameBPosition, frameAPosition);
      frameAToB.setZ(0.0); // Keep the custom frame Z-up.
      if (frameAToB.normSquared() < 1.0e-8)
         return;

      frameAToB.normalize();

      RigidBodyTransform frameTransform = transform.getValueAndModify();
      frameTransform.getTranslation().scaleAdd(-getDistance(), frameAToB, frameBPosition);
      frameTransform.getRotation().setToYawOrientation(Math.atan2(frameAToB.getY(), frameAToB.getX()));
      referenceFrame.update();
   }

   @Override
   public boolean isStable()
   {
      return scene.findFrameByName(getFrameA()) != null && scene.findFrameByName(getFrameB()) != null;
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);
      message.getPersistentDetection().setIsStable(isStable());
   }
}
