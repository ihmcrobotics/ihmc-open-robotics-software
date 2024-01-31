package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImColor;
import imgui.ImVec2;
import imgui.extension.imnodes.ImNodes;
import imgui.extension.imnodes.flag.ImNodesColorStyle;
import imgui.internal.ImGui;
import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

/**
 * @deprecated Here for reference. We are going for a newer design.
 */
public class RDXImNodesTreeNode
{
   private final RDXBehaviorUIInterface behaviorNodeUI;
   private final int nodeID;
   private final MutableInt pinIndex;
   private final ImGuiMovingPlot tickPlot = new ImGuiMovingPlot("ticks", 1000, 230, 15);
   private final ImVec2 size = new ImVec2(0.0f, 0.0f);
   private boolean nodeSizeHasChanged = false;
   private boolean hasPrintedSizeWarning = false;
   private final ArrayList<RDXImNodesTreeNode> children = new ArrayList<>();

   public RDXImNodesTreeNode(RDXBehaviorUIInterface behaviorNodeUI, int nodeID, MutableInt pinIndex)
   {
      this.behaviorNodeUI = behaviorNodeUI;
      this.nodeID = nodeID;
      this.pinIndex = pinIndex;

      for (RDXBehaviorUIInterface childUINode : behaviorNodeUI.getUIChildren())
      {
         children.add(new RDXImNodesTreeNode(childUINode, nodeID + 1, pinIndex));
      }
   }

   public void render(int parentPinIndex, ArrayList<Pair<Integer, Integer>> links)
   {
//      double timeSinceLastTick = behaviorNodeUI.getState().getTimeSinceLastTick();
//      boolean isTickRecent = behaviorNodeUI.getState().hasBeenTicked() && timeSinceLastTick < 5.0;
      double timeSinceLastTick = 1.0; // FIXME
      boolean isTickRecent = false;

      int color;
//      if (behaviorNodeUI.getState().getStatus() == BehaviorTreeNodeStatus.SUCCESS && isTickRecent)
//      {
//         color = ImColor.floatToColor(0.19607843f, 0.658823529412f, 0.321568627451f);
//      }
//      else if (behaviorNodeUI.getState().getStatus() == BehaviorTreeNodeStatus.FAILURE && isTickRecent)
//      {
//         color = ImColor.floatToColor(0.658823529412f, 0.19607843f, 0.19607843f);
//      }
//      else
//      {
         color = ImColor.floatToColor(0.19607843f, 0.321568627451f, 0.921568627451f);
//      }

      ImNodes.pushColorStyle(ImNodesColorStyle.NodeOutline, color);

      ImNodes.beginNode(nodeID);

      String nodeName;
      String nodeType;

      String name = behaviorNodeUI.getDefinition().getDescription();
      // TODO: Fix or delete
//      if (behaviorNodeUI instanceof SequenceNode)
//      {
//         nodeName = (name != null ? name : "Sequence Node");
//         nodeType = "[->]";
//      }
//      else if (behaviorNodeUI instanceof FallbackNode)
//      {
//         nodeName = (name != null ? name : "Fallback Node");
//         nodeType = "[?]";
//      }
//      else if (behaviorNodeUI instanceof AsynchronousActionNode)
//      {
//         nodeName = (name != null ? name : "Asynchronous Node");
//         nodeType = "[Async]";
//      }
//      else if (behaviorNodeUI instanceof BehaviorTreeNodeState)
//      {
//         nodeName = (name != null ? name : "Node");
//         nodeType = "[Action]";
//      }
//      else if (behaviorNodeUI instanceof BehaviorTreeCondition)
//      {
//         nodeName = (name != null ? name : "Condition");
//         nodeType = "[Condition]";
//      }
//      else if (behaviorNodeUI instanceof OneShotAction)
//      {
//         nodeName = (name != null ? name : "One Shot Action");
//         nodeType = "[OneShot]";
//      }
//      else if (behaviorNodeUI instanceof AlwaysSuccessfulAction)
//      {
//         nodeName = (name != null ? name : "Always Successful Action");
//         nodeType = "[Success]";
//      }
//      else
      {
         nodeName = (name != null ? name : "Behavior " + nodeID);
         nodeType = "[*]";
      }

      ImGui.textUnformatted(nodeType + " " + nodeName);

      if (parentPinIndex != -1)
      {
         ImGui.sameLine();

         ImNodes.beginInputAttribute(pinIndex.getValue());
         ImGui.dummy(1.0f, 1.0f);
         ImNodes.endInputAttribute();

         links.add(new Pair<>(parentPinIndex, pinIndex.getAndIncrement()));
      }

      int nodePinIndex = pinIndex.getValue();
      for (RDXBehaviorUIInterface child : behaviorNodeUI.getUIChildren())
      {
         ImGui.sameLine();

         ImNodes.beginOutputAttribute(pinIndex.getAndIncrement());
         ImGui.dummy(1f, 1f);
         ImNodes.endOutputAttribute();
      }

      double tickPeriod = 0.2;
      double recentTickWindow = tickPeriod * 0.75;
//      boolean tickedThisFrame = behaviorNodeUI.getState().hasBeenTicked() && timeSinceLastTick < recentTickWindow;
//      boolean tickedRecently = behaviorNodeUI.getState().hasBeenTicked() && timeSinceLastTick < 1.0;
      boolean tickedThisFrame = behaviorNodeUI.getState().getIsActive();
      boolean tickedRecently = behaviorNodeUI.getState().getIsActive();
      BehaviorTreeNodeStatus status = null;
      tickPlot.setNextValue(tickedThisFrame && status != null ? (float) (status.ordinal()) : Float.NaN);
      tickPlot.calculate(status != null && tickedRecently ? status.name() : "", true);

      if (status != null)
      {
         ImGui.text(String.format("Last tick: %.2f s ago", timeSinceLastTick));
         ImGui.sameLine();
         ImGui.text("Last status: " + status.name());
      }
      else
      {
         ImGui.text("Not yet ticked.");
      }

      behaviorNodeUI.renderTreeNodeImGuiWidgets();

      ImNodes.endNode();
      ImNodes.popColorStyle();

      float previousSizeX = size.x;
      float previousSizeY = size.y;
      ImNodes.getNodeDimensions(nodeID, size);

      if (previousSizeX > 0.0f)
      {
         // Warn the user when the node size changes due to the node implementation's ImGui render because it's bad to do.
         if (Math.abs(size.x - previousSizeX) > 0.5f || Math.abs(size.y - previousSizeY) > 0.5f && !nodeSizeHasChanged)
         {
            nodeSizeHasChanged = true;
            if (!hasPrintedSizeWarning)
            {
               hasPrintedSizeWarning = true;
               LogTools.warn("Node size has changed for node " + nodeID + " (" + nodeName
                             + ") - when implementing renderTreeNode(), ensure the node renders at a fixed size.");
            }
         }
      }
   }

   public ImVec2 getSize()
   {
      return size;
   }

   public RDXBehaviorUIInterface getBehaviorNodeUI()
   {
      return behaviorNodeUI;
   }

   public ArrayList<RDXImNodesTreeNode> getChildren()
   {
      return children;
   }

   public int getNodeID()
   {
      return nodeID;
   }
}
