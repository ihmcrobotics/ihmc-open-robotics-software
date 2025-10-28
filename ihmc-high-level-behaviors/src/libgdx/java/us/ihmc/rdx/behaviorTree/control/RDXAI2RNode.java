package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;

public class RDXAI2RNode extends RDXBehaviorTreeNode<AI2RNodeState, AI2RNodeDefinition>
{
   public RDXAI2RNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new AI2RNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      super.renderNodeSettingsWidgets();

      ImGui.text("AI2R Node");
   }
}
