package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneDoorFrameExecutor;
import us.ihmc.commons.UnitConversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

public class RDXBehaviorTreeSceneDoorFrame extends RDXBehaviorTreeSceneObject
{
   private RDXBehaviorTreeSceneDoorPanel doorPanel;

   private String panelNameID = "Unknown";
   private long pointsInCapsule = 0;

   public RDXBehaviorTreeSceneDoorFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);

      model = RDXModelBuilder.buildModel(modelBuilder -> modelBuilder.addCapsule(0.4, 0.05, new Vector3D(0.8 + 0.16, 0.0, 0.0),
                                                                                 LibGDXTools.toLibGDX(ColorDefinitions.GreenYellow())));
      modelInstance = new RDXModelInstance(model);
      modelInstance.setOpacity(0.3f);
   }

   public void setDoorPanel(RDXBehaviorTreeSceneDoorPanel doorPanel)
   {
      this.doorPanel = doorPanel;
      panelNameID = doorPanel.getName() + " " + doorPanel.getID();
   }

   @Override
   public void renderDetectionInfo()
   {
      ImGui.beginDisabled(!doorPanel.getPersistentDetection().getIsStable());
      ImGui.text("Door panel: %s".formatted(panelNameID));
      ImGui.endDisabled();

      double angle = doorPanel.getTransformToWorld().getRotation().distance(transform.getValueReadOnly().getRotation());

      String doorType = "Unknown";
      if (angle > Math.toRadians(10.0)) // TODO: Will need to flip based on hinge side, this is for left hinge side
         doorType = "Pull";
      else if (angle < -Math.toRadians(10.0))
         doorType = "Push";

      ImGui.beginDisabled(!getPersistentDetection().getIsStable());
      ImGui.text("Door type: %s".formatted(doorType));
      ImGui.text("Door open angle: %.2f%s".formatted(Math.toDegrees(angle), UnitConversions.DEGREE_SYMBOL));
      ImGui.text("Points in capsule: %d / %d".formatted(pointsInCapsule, BehaviorTreeSceneDoorFrameExecutor.MINIMUM_POINTS));
      ImGui.endDisabled();
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      pointsInCapsule = message.getPointsInCapsule();
   }
}
