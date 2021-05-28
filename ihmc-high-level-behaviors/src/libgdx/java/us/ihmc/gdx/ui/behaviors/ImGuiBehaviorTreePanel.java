package us.ihmc.gdx.ui.behaviors;

import imgui.ImColor;
import imgui.extension.imnodes.ImNodes;
import imgui.extension.imnodes.flag.ImNodesColorStyle;
import imgui.internal.ImGui;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNodeBasics;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.CheckableBehaviorTreeNode;
import us.ihmc.gdx.imgui.ImGuiTools;

public class ImGuiBehaviorTreePanel
{
   private final String windowName;
   private final BehaviorTreeControlFlowNodeBasics tree;

   private static int nodeIndex = 0;

   public ImGuiBehaviorTreePanel(BehaviorTreeControlFlowNodeBasics tree, String name)
   {
      windowName = ImGuiTools.uniqueLabel(getClass(), name + " tree");
      this.tree = tree;
   }

   public void renderAsWindow()
   {
      ImGui.begin(windowName);
      renderWidgetsOnly();
      ImGui.end();
   }

   public void renderWidgetsOnly()
   {
      ImNodes.beginNodeEditor();

      nodeIndex = 0;
      renderNodeAndChildren(tree);

      ImNodes.endNodeEditor();
   }

   private void renderNodeAndChildren(BehaviorTreeNode node) {
      if (node instanceof CheckableBehaviorTreeNode) {
         if (((CheckableBehaviorTreeNode) node).getPreviousStatus() == BehaviorTreeNodeStatus.SUCCESS)
            ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#32a852"));
         else if (((CheckableBehaviorTreeNode) node).getPreviousStatus() == BehaviorTreeNodeStatus.FAILURE)
            ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#a83232"));
         else
            ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#3452eb"));
      }

      ImNodes.beginNode(nodeIndex++);

      ImNodes.beginNodeTitleBar();
      switch(node.getClass().getSimpleName()) {
         case "SequenceNode": ImGui.textUnformatted("Sequence Node"); break;
         case "FallbackNode": ImGui.textUnformatted("Fallback Node"); break;
         default: ImGui.textUnformatted("Behavior " + (nodeIndex - 1)); break;
      }
      ImNodes.endNodeTitleBar();

      ImGui.dummy(40f, 20f);

      ImNodes.endNode();

      if (node instanceof CheckableBehaviorTreeNode) {
         ImNodes.popColorStyle();
      }

      if (node instanceof BehaviorTreeControlFlowNodeBasics)
      {
         for (BehaviorTreeNode child : ((BehaviorTreeControlFlowNodeBasics) node).getChildren())
            renderNodeAndChildren(child);
      }

   }
}
