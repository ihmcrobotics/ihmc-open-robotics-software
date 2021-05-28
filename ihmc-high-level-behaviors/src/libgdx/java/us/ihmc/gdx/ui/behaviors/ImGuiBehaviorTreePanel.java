package us.ihmc.gdx.ui.behaviors;

import imgui.ImColor;
import imgui.extension.imnodes.ImNodes;
import imgui.extension.imnodes.flag.ImNodesColorStyle;
import imgui.internal.ImGui;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

public class ImGuiBehaviorTreePanel
{
   private final String windowName;
   private final BehaviorTreeControlFlowNodeBasics tree;

   private int nodeIndex = 0;
   private int pinIndex = 0;
   private int linkIndex = 0;

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
      ArrayList<Pair<Integer, Integer>> links = new ArrayList<>();
      renderNodeAndChildren(tree, -1, links);
      renderLinks(links);

      ImNodes.endNodeEditor();
   }

   public void renderLinks(ArrayList<Pair<Integer, Integer>> links) {
      for (Pair<Integer, Integer> p : links) {
         ImNodes.link(linkIndex++, p.getFirst(), p.getSecond());
      }
   }

   private void renderNodeAndChildren(BehaviorTreeNodeBasics node, int parentPin, ArrayList<Pair<Integer, Integer>> links) {
      long timeSinceLastTick = -1;

      if (node instanceof BehaviorTreeNode) {
         timeSinceLastTick = System.currentTimeMillis() - ((BehaviorTreeNode) node).lastTickMillis();
         boolean isTickRecent = timeSinceLastTick < 5000;

         if (((BehaviorTreeNode) node).getPreviousStatus() == BehaviorTreeNodeStatus.SUCCESS && isTickRecent)
            ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#32a852"));
         else if (((BehaviorTreeNode) node).getPreviousStatus() == BehaviorTreeNodeStatus.FAILURE && isTickRecent)
            ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#a83232"));
         else
            ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#3452eb"));
      }

      ImNodes.beginNode(nodeIndex);

      ImNodes.beginNodeTitleBar();
      String name = null;
      if (node instanceof BehaviorTreeNode)
         name = ((BehaviorTreeNode) node).getName();

      switch(node.getClass().getSimpleName()) {
         case "SequenceNode": ImGui.textUnformatted("[->] " + (name != null ? name : "Sequence Node")); break;
         case "FallbackNode": ImGui.textUnformatted("[F] " + (name != null ? name : "Fallback Node")); break;
         case "LoopSequenceNode": ImGui.textUnformatted("[o] " + (name != null ? name : "Loop Sequence Node")); break;
         case "OneShotAction": ImGui.textUnformatted("[1] " + (name != null ? name : "One Shot Action")); break;
         case "AlwaysSuccessfulAction": ImGui.textUnformatted("[A] " + (name != null ? name : "Always Successful Action")); break;
         default: ImGui.textUnformatted(name != null ? name : "Behavior " + nodeIndex); break;
      }
      ImNodes.endNodeTitleBar();

      if (timeSinceLastTick > -1)
      {
         if (((BehaviorTreeNode) node).getPreviousStatus() != null)
            ImGui.textColored(0xFFFFFFFF, "Last tick: " + (timeSinceLastTick < 1000 ? "< 1s" : timeSinceLastTick / 1000 + "s") + " ago");
         else
            ImGui.textColored(0xFFFFFFFF,"Not yet ticked.");
      }

      ImGui.dummy(120f, 20f);

      if (parentPin != -1) {
         ImNodes.beginInputAttribute(pinIndex);
         ImNodes.endInputAttribute();

         links.add(new Pair<>(parentPin, pinIndex++));
      }

      int nodePinIndex = pinIndex;
      if (node instanceof BehaviorTreeControlFlowNode) {
         for (BehaviorTreeNodeBasics child : ((BehaviorTreeControlFlowNode) node).getChildren()) {
            ImNodes.beginOutputAttribute(pinIndex++);
            ImNodes.endOutputAttribute();
         }
      }

      ImNodes.endNode();
      nodeIndex++;

      if (node instanceof BehaviorTreeNode) {
         ImNodes.popColorStyle();
      }

      if (node instanceof BehaviorTreeControlFlowNode)
      {
         for (BehaviorTreeNodeBasics child : ((BehaviorTreeControlFlowNode) node).getChildren())
            renderNodeAndChildren(child, nodePinIndex++, links);
      }

   }
}
