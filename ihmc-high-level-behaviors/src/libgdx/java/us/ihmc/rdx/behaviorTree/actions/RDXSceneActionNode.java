package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionNodeState;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXSceneActionNode extends RDXActionNode<SceneActionNodeState, SceneActionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString imYOLOModelName = new ImString(256);
   private final ImInt currentObjectType = new ImInt(0);
   private final String[] objectTypeNames;

   public RDXSceneActionNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new SceneActionNodeState(id, rootNode.getState()), rootNode);

      IsaacROSFoundationPoseObject[] values = IsaacROSFoundationPoseObject.values();
      objectTypeNames = new String[values.length];
      for (int i = 0; i < values.length; i++)
      {
         objectTypeNames[i] = values[i].titleCaseName;
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      imYOLOModelName.set(definition.getYoloModelName());
      if (ImGui.inputText(labels.get("YOLO Model Name"), imYOLOModelName, ImGuiInputTextFlags.EnterReturnsTrue))
         definition.setYoloModelName(imYOLOModelName.get());

      if (ImGui.checkbox(labels.get("Use FoundationPose"), definition.getUseFoundationPose()))
         definition.setUseFoundationPose(!definition.getUseFoundationPose());

      ImGui.pushItemWidth(200.0f);
      currentObjectType.set(definition.getObjectType().ordinal());
      if (ImGui.combo(labels.get("Object Type"), currentObjectType, objectTypeNames))
      {
         definition.setObjectType(IsaacROSFoundationPoseObject.values()[currentObjectType.get()]);
      }
      ImGui.popItemWidth();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Scene Action";
   }
}
