package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneDoorFrameExecutor;
import us.ihmc.behaviors.simulation.door.DoorModelParameters;
import us.ihmc.commons.UnitConversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

public class RDXBehaviorTreeSceneDoorFrame extends RDXBehaviorTreeSceneObject
{
   private RDXBehaviorTreeSceneDoorPanel doorPanel;

   private boolean doorPanelStateKnown = false;
   private String panelNameID = "Unknown";
   private String doorType = "Unknown";
   private double panelToFrameAngle = Double.NaN;
   private long pointsInCapsule = 0;

   public RDXBehaviorTreeSceneDoorFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);

      final double height = DoorModelParameters.DOOR_PANEL_HEIGHT;
      final double radius = BehaviorTreeSceneDoorFrameExecutor.SEARCH_CAPSULE_RADIUS;

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

   public void setDoorPanel(RDXBehaviorTreeSceneDoorPanel doorPanel)
   {
      this.doorPanel = doorPanel;
      panelNameID = doorPanel.getName() + " " + doorPanel.getID();
   }

   @Override
   public void update()
   {
      super.update();

      if (doorPanel != null)
      {
         doorPanelStateKnown = doorPanel.getDoorPanelPersistentDetection().getIsStable() && doorPanel.getPersistentDetection().getIsStable();

         if (getTransformToWorld().hasTranslation() && getTransformToWorld().hasRotation())
         {
            panelToFrameAngle = Math.PI - (doorPanel.getTransformToWorld().getRotation().getYaw() - getTransformToWorld().getRotation().getYaw());

            if (panelToFrameAngle > 3.0 * BehaviorTreeSceneDoorFrameExecutor.SEARCH_ANGLE_INCREMENT)
               doorType = "Pull";
            else if (panelToFrameAngle < -3.0 * BehaviorTreeSceneDoorFrameExecutor.SEARCH_ANGLE_INCREMENT)
               doorType = "Push";
         }
      }
   }

   @Override
   public void renderDetectionInfo()
   {
      ImGui.beginDisabled(!doorPanelStateKnown);
      ImGui.text("Door panel: %s".formatted(panelNameID));
      ImGui.text("Door type: %s".formatted(doorType));
      ImGui.text("Door open angle: %.2f%s".formatted(Math.toDegrees(Math.abs(panelToFrameAngle)), UnitConversions.DEGREE_SYMBOL));
      ImGui.endDisabled();
      ImGui.beginDisabled(pointsInCapsule < BehaviorTreeSceneDoorFrameExecutor.MINIMUM_POINTS);
      ImGui.text("Points in capsule: %d (minimum %d)".formatted(pointsInCapsule, BehaviorTreeSceneDoorFrameExecutor.MINIMUM_POINTS));
      ImGui.endDisabled();
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      pointsInCapsule = message.getPointsInCapsule();
   }
}
