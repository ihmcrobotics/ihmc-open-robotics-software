package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.List;
import java.util.function.LongSupplier;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel panel = new RDXPanel("Scene", this::renderImGuiWidgets);
   private final ImBoolean mustard = new ImBoolean(false);

   private final List<RDXBehaviorTreeSceneObject> objects;

   private boolean needToInitializePlacementHeight = false;
   private RDXBehaviorTreeSceneObject beingPlaced;

   public RDXBehaviorTreeScene(CRDTInfo crdtInfo, LongSupplier idSupplier, ROS2SyncedRobotModel syncedRobot, RDXBaseUI baseUI, RDXPanel parentPanel)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.baseUI = baseUI;

      this.objects = (List) super.objects;

      parentPanel.addChild(panel);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   public void update()
   {
      for (RDXBehaviorTreeSceneObject object : objects)
         object.update();
   }

   private void processImGui3DViewInput(ImGui3DViewInput input)
   {
      if (beingPlaced != null)
      {
         if (needToInitializePlacementHeight) // Initialize placed height until scene collisions take back over
         {
            needToInitializePlacementHeight = false;
            input.setLastZCollision(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ());
         }
      }
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Add virtual:");
      ImGui.indent();
      if (ImGuiTools.textWithUnderlineOnHover("Mustard") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
      {
         beingPlaced = new RDXBehaviorTreeSceneObject(idSupplier.getAsLong(), crdtInfo, IsaacROSFoundationPoseObject.MUSTARD.name(), baseUI);
         objects.add(beingPlaced);
         objectsModifiable.modify();
         needToInitializePlacementHeight = true;
      }
      ImGui.unindent();

      ImGui.text("Objects:");
      ImGui.indent();
      RDXBehaviorTreeSceneObject remove = null;
      for (int i = 0; i < objects.size(); i++)
      {
         RDXBehaviorTreeSceneObject object = objects.get(i);
         ImGui.text("%s ID: %d".formatted(object.getName(), object.getID()));
         ImGui.sameLine();
         ImGui.pushStyleColor(ImGuiCol.Button, ImGuiTools.DARK_RED);
         if (ImGui.button(labels.get("X", i)))
            remove = object;
         ImGui.popStyleColor();
      }
      ImGui.unindent();

      if (remove != null)
      {
         objects.remove(remove);
         objectsModifiable.modify();
         remove.destroy();
      }
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(BehaviorTreeSceneObjectStateMessage message)
   {
      return new RDXBehaviorTreeSceneObject(message.getId(), crdtInfo, message.getTypeAsString(), baseUI);
   }
}
