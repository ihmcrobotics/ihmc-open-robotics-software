package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import com.badlogic.gdx.graphics.Color;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXBehaviorTreeSceneDoorFrame extends RDXBehaviorTreeSceneObject
{
   private final Pose3D hingePostPose;
   private final Pose3D latchPostPose;

   public RDXBehaviorTreeSceneDoorFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);

      hingePostPose = new Pose3D();
      hingePostPose.setToNaN();

      latchPostPose = new Pose3D();
      latchPostPose.setToNaN();

      updateModel();
   }

   private void updateModel()
   {
      final double height = DoorModelParameters.DOOR_PANEL_HEIGHT;
      final double radius = 0.1;

      if (model != null)
      {
         model.dispose();
         model = null;
      }

      if (hingePostPose.containsNaN() || latchPostPose.containsNaN())
         return;

      RigidBodyTransform hingeToLatchTransform = new RigidBodyTransform(hingePostPose);
      hingeToLatchTransform.multiplyInvertOther(latchPostPose);

      model = RDXModelBuilder.buildModel(modelBuilder ->
      {
         Vector3D hingePostModelTranslation = new Vector3D(0.0, 0.0, -0.5 * height);
         Vector3D latchPostModelTranslation = new Vector3D(hingeToLatchTransform.getTranslation());
         latchPostModelTranslation.subZ(0.5 * height);

         modelBuilder.addCylinder(height, radius, hingePostModelTranslation, Color.BLUE);
         modelBuilder.addCylinder(height, radius, latchPostModelTranslation, hingeToLatchTransform.getRotation(), Color.RED);
      });

      modelInstance = new RDXModelInstance(model);
      modelInstance.setOpacity(0.2f);
   }

   @Override
   public void update()
   {
      super.update();

      modelInstance.setTransformToWorldFrame(new RigidBodyTransform(hingePostPose));
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      if (!hingePostPose.epsilonEquals(message.getHingePostPose(), 1E-3) || !latchPostPose.epsilonEquals(message.getHingePostPose(), 1E-3))
      {
         hingePostPose.set(message.getHingePostPose());
         latchPostPose.set(message.getLatchPostPose());
         updateModel();
      }
   }
}
