package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import imgui.ImGui;
import us.ihmc.commons.UnitConversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import static behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage.*;

public class RDXBehaviorTreeSceneDoorFrame extends RDXBehaviorTreeSceneObject
{
   private RDXBehaviorTreeSceneDoorPanel doorPanel;

   private String panelNameID = "Unknown";
   private RobotSide hingeSide;
   private long latchPostPoints = 0;
   private long hingeRecessPoints = 0;
   private String doorType = "Unknown";
   private float doorOpenAngle = Float.NaN;

   public RDXBehaviorTreeSceneDoorFrame(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);
   }

   public void setDoorPanel(RDXBehaviorTreeSceneDoorPanel doorPanel)
   {
      this.doorPanel = doorPanel;
      panelNameID = doorPanel.getName() + " " + doorPanel.getID();
   }

   @Override
   public void update()
   {
      if (model == null && latchPostPoints > getMinPostPoints()) // Don't show until valid
      {
         model = RDXModelBuilder.buildModel(modelBuilder ->
         {
            modelBuilder.addCapsule(0.6, 0.05, new Vector3D(0.8 + 0.16, 0.0, 0.0), LibGDXTools.toLibGDX(ColorDefinitions.GreenYellow()));
            modelBuilder.addCapsule(0.6, 0.05, new Vector3D(-0.1, 0.1, 0.0), LibGDXTools.toLibGDX(ColorDefinitions.GreenYellow()));
            modelBuilder.addCapsule(0.6, 0.05, new Vector3D(-0.1, -0.1, 0.0), LibGDXTools.toLibGDX(ColorDefinitions.GreenYellow()));
         });
         modelInstance = new RDXModelInstance(model);
         modelInstance.setOpacity(0.3f);
      }

      super.update();
   }

   @Override
   public void renderDetectionInfo()
   {
      ImGui.beginDisabled(!doorPanel.getPersistentDetection().getIsStable());
      ImGui.text("Door panel: %s".formatted(panelNameID));
      ImGui.endDisabled();

      ImGui.beginDisabled(!getPersistentDetection().getIsStable());
      ImGui.text("Hinge side: %s".formatted(hingeSide == null ? "null" : hingeSide.getPascalCaseName()));
      ImGui.text("Door type: %s".formatted(doorType));
      ImGui.text("Door open angle: %.2f%s".formatted(Math.toDegrees(doorOpenAngle), UnitConversions.DEGREE_SYMBOL));
      ImGui.text("Latch post points: %d / %d".formatted(latchPostPoints, getMinPostPoints()));
      ImGui.endDisabled();
      ImGui.beginDisabled(hingeRecessPoints <= getMinRecessPoints());
      ImGui.text("Hinge recess points: %d / %d".formatted(hingeRecessPoints, getMinRecessPoints()));
      ImGui.endDisabled();
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      hingeSide = RobotSide.fromByte(message.getHingeSide());
      latchPostPoints = message.getLatchPostPoints();
      hingeRecessPoints = message.getHingeRecessPoints();
      doorType = switch (message.getDoorType())
      {
         case DOOR_TYPE_PULL -> "Pull";
         case DOOR_TYPE_PUSH -> "Push";
         default -> "Unknown";
      };
      doorOpenAngle = message.getDoorOpenAngle();
   }
}
