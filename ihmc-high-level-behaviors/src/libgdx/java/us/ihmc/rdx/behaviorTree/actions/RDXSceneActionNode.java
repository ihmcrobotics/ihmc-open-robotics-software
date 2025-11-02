package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXSceneActionNode extends RDXActionNode<SceneActionNodeState, SceneActionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXSceneActionNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new SceneActionNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.text("Scene action node");
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Scene Action";
   }
}
