package us.ihmc.gdx.ui.behaviors;

import imgui.ImVec2;
import imgui.extension.nodeditor.NodeEditor;
import imgui.extension.nodeditor.NodeEditorContext;
import imgui.extension.nodeditor.flag.NodeEditorPinKind;
import imgui.extension.nodeditor.flag.NodeEditorStyleColor;
import imgui.extension.nodeditor.flag.NodeEditorStyleVar;
import imgui.internal.ImGui;
import org.abego.treelayout.Configuration;
import org.abego.treelayout.NodeExtentProvider;
import org.abego.treelayout.TreeLayout;
import org.abego.treelayout.util.DefaultTreeForTreeLayout;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.FormattingTools;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class ImGuiNodeEditorBehaviorTreePanel
{
   private final String windowName;

   private int nodeIndex = 0;
   private int pinIndex = 0;
   private int linkIndex = 0;
   private boolean firstRun = true;
   private final HashMap<Integer, ImGuiMovingPlot> tickPlots = new HashMap<>();

   private final HashMap<Integer, ImVec2> nametagSize = new HashMap<>();
   private final HashMap<Integer, ImVec2> nodeSize = new HashMap<>();
   private final HashMap<Integer, ImVec2> nodeSizeCorrection = new HashMap<>();

   private NodeEditorContext context = null;

   /***
    * Note that create() and dispose() should be called for creating and destroying the context.
    *
    * @param name The name of the window
    */
   public ImGuiNodeEditorBehaviorTreePanel(String name)
   {
      windowName = ImGuiTools.uniqueLabel(getClass(), name);
   }

   private void resetNodeIndex(GDXBehaviorUIInterface tree) {
      nodeIndex = tree.generateUID();
   }

   public void create()
   {
      context = NodeEditor.createEditor();
      NodeEditor.setCurrentEditor(context);

      NodeEditor.pushStyleColor(NodeEditorStyleColor.Bg, 0.9f, 0.9f, 0.9f, 1);
      NodeEditor.pushStyleColor(NodeEditorStyleColor.NodeBg, 0.8f, 0.8f, 0.8f, 1);
      NodeEditor.pushStyleVar(NodeEditorStyleVar.NodeBorderWidth, 5);
   }

   public void renderAsWindow(GDXBehaviorUIInterface tree)
   {
      ImGui.begin(windowName);
      renderWidgetsOnly(tree);
      ImGui.end();
   }

   public void renderWidgetsOnly(GDXBehaviorUIInterface tree)
   {
      NodeEditor.setCurrentEditor(context);
      ImGui.pushFont(ImGuiTools.getNodeFont());
      NodeEditor.begin(windowName);
      resetNodeIndex(tree);
      ArrayList<Pair<Integer, Integer>> links = new ArrayList<>();
      renderNodeAndChildren(tree, -1, links);
      renderLinks(links);

      if (firstRun)
      {
         boolean repositionNodes = true;
         for (int id : nodeSize.keySet()) {
            if (NodeEditor.getNodePositionX(id) != 0.0f || NodeEditor.getNodePositionY(id) != 0.0f)
            {
               repositionNodes = false;
               break;
            }
         }

         if (repositionNodes)
            layoutNodes(tree);
      }

      NodeEditor.end();
      ImGui.popFont();
      firstRun = false;
   }

   private void constructAbegoTree(GDXBehaviorUIInterface tree, DefaultTreeForTreeLayout<GDXBehaviorUIInterface> layout) {
      if (tree == null)
         return;

      for (GDXBehaviorUIInterface child : tree.getUIChildren()) {
         layout.addChild(tree, child);
         constructAbegoTree(child, layout);
      }
   }

   private int getIndexOfNodeInternal(GDXBehaviorUIInterface node, GDXBehaviorUIInterface root) {
      if (root.equals(node))
         return nodeIndex;
      else {
         for (GDXBehaviorUIInterface child : root.getUIChildren()) {
            nodeIndex++;
            int val = getIndexOfNodeInternal(node, child);

            if (val != -1)
               return val;
         }
      }

      return -1;
   }

   private int getIndexOfNode(GDXBehaviorUIInterface node, GDXBehaviorUIInterface tree) {
      resetNodeIndex(tree);
      return getIndexOfNodeInternal(node, tree);
   }

   public void layoutNodes(GDXBehaviorUIInterface tree)
   {
      DefaultTreeForTreeLayout<GDXBehaviorUIInterface> treeForLayout = new DefaultTreeForTreeLayout<>(tree);
      constructAbegoTree(tree, treeForLayout);

      TreeLayout<GDXBehaviorUIInterface> layout = new TreeLayout<GDXBehaviorUIInterface>(treeForLayout, new NodeExtentProvider<GDXBehaviorUIInterface>()
      {

         @Override
         public double getWidth(GDXBehaviorUIInterface gdxBehaviorUIInterface)
         {
            int index = getIndexOfNode(gdxBehaviorUIInterface, tree);
            return nodeSize.get(index).x;
         }

         @Override
         public double getHeight(GDXBehaviorUIInterface gdxBehaviorUIInterface)
         {
            int index = getIndexOfNode(gdxBehaviorUIInterface, tree);
            return nodeSize.get(index).y;
         }
      }, new Configuration<GDXBehaviorUIInterface>()
      {
         @Override
         public Location getRootLocation()
         {
            return Location.Top;
         }

         @Override
         public AlignmentInLevel getAlignmentInLevel()
         {
            return AlignmentInLevel.AwayFromRoot;
         }

         @Override
         public double getGapBetweenLevels(int nextLevel)
         {
            return 50;
         }

         @Override
         public double getGapBetweenNodes(GDXBehaviorUIInterface node1, GDXBehaviorUIInterface node2)
         {
            return 25;
         }
      });

      Map<GDXBehaviorUIInterface, Rectangle2D.Double> map = layout.getNodeBounds();
      for (GDXBehaviorUIInterface node : map.keySet()) {
         int index = getIndexOfNode(node, tree);
         Rectangle2D.Double pos = map.get(node);

         NodeEditor.setNodePosition(index, (float)pos.x, (float)pos.y);
      }
   }

   public void renderLinks(ArrayList<Pair<Integer, Integer>> links)
   {
      for (Pair<Integer, Integer> p : links)
      {
         NodeEditor.link(linkIndex++, p.getFirst(), p.getSecond(), 0, 0, 0, 1, 2);
      }
   }

   private void renderNodeAndChildren(GDXBehaviorUIInterface node, int parentPin, ArrayList<Pair<Integer, Integer>> links)
   {
      long timeSinceLastTick = -1;

      timeSinceLastTick = System.currentTimeMillis() - node.getLastTickMillis();
      boolean isTickRecent = timeSinceLastTick < 5000;

      if (node.getPreviousStatus() == BehaviorTreeNodeStatus.SUCCESS && isTickRecent)
      {
         NodeEditor.pushStyleColor(NodeEditorStyleColor.NodeBorder, 0.19607843f, 0.658823529412f, 0.321568627451f, 1);
      }
      else if (node.getPreviousStatus() == BehaviorTreeNodeStatus.FAILURE && isTickRecent)
      {
         NodeEditor.pushStyleColor(NodeEditorStyleColor.NodeBorder, 0.658823529412f, 0.19607843f, 0.19607843f, 1);
      }
      else
      {
         NodeEditor.pushStyleColor(NodeEditorStyleColor.NodeBorder, 0.19607843f, 0.321568627451f, 0.921568627451f, 1);
      }

      NodeEditor.beginNode(nodeIndex);

      String nodeName;
      String nodeType;

      String name = node.getName();
      if (node.getType().equals(SequenceNode.class))
      {
         nodeName = (name != null ? name : "Sequence Node");
         nodeType = "[->]";
      }
      else if (node.getType().equals(FallbackNode.class))
      {
         nodeName = (name != null ? name : "Fallback Node");
         nodeType = "[?]";
      }
      else if (node.getType().equals(AsynchronousActionNode.class))
      {
         nodeName = (name != null ? name : "Asynchronous Node");
         nodeType = "[Async]";
      }
      else if (node.getType().equals(BehaviorTreeAction.class))
      {
         nodeName = (name != null ? name : "Action");
         nodeType = "[Action]";
      }
      else if (node.getType().equals(BehaviorTreeCondition.class))
      {
         nodeName = (name != null ? name : "Condition");
         nodeType = "[Condition]";
      }
      else if (node.getType().equals(OneShotAction.class))
      {
         nodeName = (name != null ? name : "One Shot Action");
         nodeType = "[OneShot]";
      }
      else if (node.getType().equals(AlwaysSuccessfulAction.class))
      {
         nodeName = (name != null ? name : "Always Successful Action");
         nodeType = "[Success]";
      }
      else
      {
         nodeName = (name != null ? name : "Behavior " + nodeIndex);
         nodeType = "[*]";
      }

      if (firstRun)
      {
         ImVec2 size = new ImVec2();
         ImGui.calcTextSize(size, nodeType + " " + nodeName);
         nametagSize.put(nodeIndex, size);
      }

      boolean shouldRender = NodeEditor.getCurrentZoom() < 1.5f || firstRun;

      if (shouldRender)
      {
         ImGui.textUnformatted(nodeType + " " + nodeName);
      }
      else
      {
         ImVec2 size = nametagSize.get(nodeIndex);
         if (size != null)
         {
            ImGui.dummy(size.x, size.y);
         }
      }

      if (parentPin != -1)
      {
         ImGui.sameLine();

         NodeEditor.beginPin(pinIndex, NodeEditorPinKind.Input);
         ImGui.dummy(1f, 1f);
         NodeEditor.endPin();

         links.add(new Pair<>(parentPin, pinIndex++));
      }

      int nodePinIndex = pinIndex;
      for (GDXBehaviorUIInterface child : node.getUIChildren())
      {
         ImGui.sameLine();

         NodeEditor.beginPin(pinIndex++, NodeEditorPinKind.Output);
         ImGui.dummy(1f, 1f);
         NodeEditor.endPin();
      }

      ImGuiMovingPlot tickPlot = tickPlots.get(nodeIndex);
      if (tickPlot == null)
      {
         tickPlot = new ImGuiMovingPlot("ticks", 1000, 230, 15);
         tickPlots.put(nodeIndex, tickPlot);
      }
      if (timeSinceLastTick > -1)
      {
         double tickPeriod = 0.2;
         double recentTickWindow = tickPeriod * 0.75;
         //         double v = UnitConversions.hertzToSeconds(Gdx.graphics.getFramesPerSecond());
         boolean tickedThisFrame = Conversions.millisecondsToSeconds(timeSinceLastTick) < recentTickWindow;
         boolean tickedRecently = Conversions.millisecondsToSeconds(timeSinceLastTick) < 1.0;
         BehaviorTreeNodeStatus status = node.getPreviousStatus();
         tickPlot.setNextValue(tickedThisFrame && status != null ? (float) (status.ordinal()) : Float.NaN);
         tickPlot.calculate(status != null && tickedRecently ? status.name() : "", shouldRender);

         if (shouldRender)
         {
            if (status != null)
            {
               ImGui.text("Last tick: " + FormattingTools.getFormattedDecimal2D(timeSinceLastTick / 1000.0) + " s ago");
               ImGui.sameLine();
               ImGui.text("Last status: " + status.name());
            }
            else
            {
               ImGui.text("Not yet ticked.");
            }

            node.renderTreeNodeImGuiWidgets();
         }
      }

      if (!shouldRender)
      {
         ImVec2 size = nodeSizeCorrection.get(nodeIndex);

         if (size != null)
         {
            ImGui.pushFont(ImGuiTools.getBigFont());

            ImVec2 textSize = new ImVec2();
            ImGui.calcTextSize(textSize, nodeName);
            ImGui.text(nodeName);

            ImGui.popFont();

            ImGui.dummy(size.x, size.y);
         }
      }

      NodeEditor.endNode();

      if (firstRun)
      {
         nodeSize.put(nodeIndex, new ImVec2(NodeEditor.getNodeSizeX(nodeIndex), NodeEditor.getNodeSizeY(nodeIndex)));
         nodeSizeCorrection.put(nodeIndex, new ImVec2(0, 0));
      } else if (!shouldRender && nodeSizeCorrection.get(nodeIndex).x == 0) {
         ImVec2 correction = nodeSizeCorrection.get(nodeIndex);
         correction.x = nodeSize.get(nodeIndex).x - ImGui.getStyle().getItemSpacingX() - 8; //8 is probably not arbitrary?
         correction.y = nodeSize.get(nodeIndex).y - NodeEditor.getNodeSizeY(nodeIndex) - 8;
      }

      nodeIndex++;

      NodeEditor.popStyleColor(1);

      for (GDXBehaviorUIInterface child : node.getUIChildren())
      {
         renderNodeAndChildren(child, nodePinIndex++, links);
      }
   }

   public void dispose()
   {
      NodeEditor.destroyEditor(context);
   }

   public String getWindowName()
   {
      return windowName;
   }
}
