package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionNodeState;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXSceneActionNode extends RDXActionNode<SceneActionNodeState, SceneActionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt currentObjectType = new ImInt(0);
   private final String[] objectTypeNames;

   public RDXSceneActionNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new SceneActionNodeState(id, rootNode.getState()), rootNode);

      IsaacROSFoundationPoseObject[] values = IsaacROSFoundationPoseObject.values();
      objectTypeNames = new String[values.length];
      for (int i = 0; i < values.length; i++)
      {
         objectTypeNames[i] = values[i].name();
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
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
