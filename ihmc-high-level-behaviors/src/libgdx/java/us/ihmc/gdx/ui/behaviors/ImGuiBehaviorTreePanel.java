package us.ihmc.gdx.ui.behaviors;

import imgui.ImColor;
import imgui.extension.imnodes.ImNodes;
import imgui.extension.imnodes.flag.ImNodesColorStyle;
import imgui.internal.ImGui;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Function;

public class ImGuiBehaviorTreePanel
{
   private final String windowName;

   private int nodeIndex = 0;
   private int pinIndex = 0;
   private int linkIndex = 0;
   private boolean firstRun = true;

   public ImGuiBehaviorTreePanel(String name)
   {
      windowName = ImGuiTools.uniqueLabel(getClass(), name);
   }

   public void renderAsWindow(BehaviorTreeControlFlowNodeBasics tree, Consumer<String> nodeRenderer, Function<String, Point2D32> positioning)
   {
      ImGui.begin(windowName);
      renderWidgetsOnly(tree, nodeRenderer, positioning);
      ImGui.end();
   }

   public void renderWidgetsOnly(BehaviorTreeControlFlowNodeBasics tree, Consumer<String> nodeRenderer, Function<String, Point2D32> positioning)
   {
      ImNodes.beginNodeEditor();
      nodeIndex = 0;
      ArrayList<Pair<Integer, Integer>> links = new ArrayList<>();
      renderNodeAndChildren(tree, nodeRenderer, positioning, -1, links);
      renderLinks(links);

      ImNodes.endNodeEditor();
      firstRun = false;
   }

   public void renderLinks(ArrayList<Pair<Integer, Integer>> links)
   {
      for (Pair<Integer, Integer> p : links)
      {
         ImNodes.link(linkIndex++, p.getFirst(), p.getSecond());
      }
   }

   private void renderNodeAndChildren(BehaviorTreeNodeBasics node,
                                      Consumer<String> nodeRenderer,
                                      Function<String, Point2D32> positioning,
                                      int parentPin,
                                      ArrayList<Pair<Integer, Integer>> links)
   {
      long timeSinceLastTick = -1;

      timeSinceLastTick = System.currentTimeMillis() - node.getLastTickMillis();
      boolean isTickRecent = timeSinceLastTick < 5000;

      if (node.getPreviousStatus() == BehaviorTreeNodeStatus.SUCCESS && isTickRecent)
         ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#32a852"));
      else if (node.getPreviousStatus() == BehaviorTreeNodeStatus.FAILURE && isTickRecent)
         ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#a83232"));
      else
         ImNodes.pushColorStyle(ImNodesColorStyle.TitleBar, ImColor.rgbToColor("#3452eb"));

      ImNodes.beginNode(nodeIndex);

      ImNodes.beginNodeTitleBar();
      String name = node.getName();
      if (node.getType().equals(SequenceNode.class))
      {
         ImGui.textUnformatted("[->] " + (name != null ? name : "Sequence Node"));
      }
      else if (node.getType().equals(FallbackNode.class))
      {
         ImGui.textUnformatted("[?] " + (name != null ? name : "Fallback Node"));
      }
      else if (node.getType().equals(AsynchronousActionNode.class))
      {
         ImGui.textUnformatted("[Async] " + (name != null ? name : "Asynchronous Node"));
      }
      else if (node.getType().equals(OneShotAction.class))
      {
         ImGui.textUnformatted("[1] " + (name != null ? name : "One Shot Action"));
      }
      else if (node.getType().equals(AlwaysSuccessfulAction.class))
      {
         ImGui.textUnformatted("[A] " + (name != null ? name : "Always Successful Action"));
      }
      else
      {
         ImGui.textUnformatted("[*] " + (name != null ? name : "Behavior " + nodeIndex));
      }
      ImNodes.endNodeTitleBar();

      if (timeSinceLastTick > -1)
      {
         if (node.getPreviousStatus() != null)
            ImGui.text("Last tick: " + FormattingTools.getFormattedDecimal2D(timeSinceLastTick / 1000.0) + " s ago");
         else
            ImGui.text("Not yet ticked.");
      }

      nodeRenderer.accept(name);

      if (firstRun)
      {
         Point2D32 position = positioning.apply(name);
         ImNodes.setNodeGridSpacePos(nodeIndex, position.getX32(), position.getY32());
      }

//      ImGui.dummy(120f, 20f);

      if (parentPin != -1)
      {
         ImNodes.beginInputAttribute(pinIndex);
         ImNodes.endInputAttribute();

         links.add(new Pair<>(parentPin, pinIndex++));
      }

      int nodePinIndex = pinIndex;
      if (node instanceof BehaviorTreeControlFlowNode)
      {
         for (BehaviorTreeNodeBasics child : ((BehaviorTreeControlFlowNode) node).getChildren())
         {
            ImNodes.beginOutputAttribute(pinIndex++);
            ImNodes.endOutputAttribute();
         }
      }

      ImNodes.endNode();
      nodeIndex++;

      if (node instanceof BehaviorTreeNode)
      {
         ImNodes.popColorStyle();
      }

      if (node instanceof BehaviorTreeControlFlowNode)
      {
         for (BehaviorTreeNodeBasics child : ((BehaviorTreeControlFlowNode) node).getChildren())
         {
            renderNodeAndChildren(child, nodeRenderer, positioning, nodePinIndex++, links);
         }
      }
   }
}
