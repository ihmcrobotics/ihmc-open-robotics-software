package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import com.badlogic.gdx.graphics.Color;
import us.ihmc.behaviors.simulation.door.DoorModelParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

public class RDXBehaviorTreeSceneDoorFrame extends RDXBehaviorTreeSceneObject
{
   public RDXBehaviorTreeSceneDoorFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);

      final double height = DoorModelParameters.DOOR_PANEL_HEIGHT;
      final double radius = 0.05;

      model = RDXModelBuilder.buildModel(modelBuilder ->
      {
         modelBuilder.addCylinder(0.5 * height, radius, new Vector3D(), Color.WHITE);
         modelBuilder.addSphere(radius, new Vector3D(0.0, 0.0, 0.5 * height), Color.WHITE);
         modelBuilder.addSphere(radius, new Vector3D(), Color.WHITE);

         Vector3D position = new Vector3D(DoorModelParameters.DOOR_PANEL_WIDTH + 0.2, 0.0, 0.0);
         modelBuilder.addCylinder(0.5 * height, radius, position, Color.WHITE);
         modelBuilder.addSphere(radius, position, Color.WHITE);
         position.addZ(0.5 * height);
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
}
