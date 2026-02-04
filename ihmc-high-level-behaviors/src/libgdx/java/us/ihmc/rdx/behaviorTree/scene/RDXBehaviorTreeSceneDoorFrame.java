package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import com.badlogic.gdx.graphics.Color;
import us.ihmc.behaviors.simulation.door.DoorModelParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

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
      final double radius = 0.07;

      if (model != null)
      {
         model.dispose();
         model = null;
      }

      if (hingePostPose.containsNaN() || latchPostPose.containsNaN())
         return;

      Pose3D latchPoseInHingePostFrame = new Pose3D(latchPostPose);
      ReferenceFrame.getWorldFrame().transformFromThisToDesiredFrame(referenceFrame, latchPoseInHingePostFrame);

      model = RDXModelBuilder.buildModel(modelBuilder ->
      {
         modelBuilder.addCylinder(0.5 * height, radius, new Vector3D(), Color.WHITE);
         modelBuilder.addSphere(radius, new Vector3D(0.0, 0.0, 0.5 * height), Color.WHITE);
         modelBuilder.addSphere(radius, new Vector3D(), Color.WHITE);

         Vector3D position = new Vector3D(latchPoseInHingePostFrame.getTranslation());
         modelBuilder.addCylinder(0.5 * height, radius, position, Color.WHITE);
         position.addZ(0.5 * height);
         modelBuilder.addSphere(radius, position, Color.WHITE);
         position.subZ(0.5 * height);
         modelBuilder.addSphere(radius, position, Color.WHITE);
      });

      modelInstance = new RDXModelInstance(model);
      modelInstance.setColor(ColorDefinitions.GreenYellow());
      modelInstance.setOpacity(0.3f);
   }

   @Override
   public void update()
   {
      super.update();

      if (modelInstance != null)
         modelInstance.setTransformToWorldFrame(new RigidBodyTransform(transform.getValueReadOnly()));
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
