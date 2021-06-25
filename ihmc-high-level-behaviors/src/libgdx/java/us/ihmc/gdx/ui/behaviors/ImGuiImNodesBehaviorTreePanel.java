package us.ihmc.gdx.ui.behaviors;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImColor;
import imgui.ImVec2;
import imgui.extension.imnodes.ImNodes;
import imgui.extension.imnodes.flag.ImNodesColorStyle;
import imgui.extension.imnodes.flag.ImNodesStyleVar;
import imgui.internal.ImGui;
import org.abego.treelayout.Configuration;
import org.abego.treelayout.NodeExtentProvider;
import org.abego.treelayout.TreeLayout;
import org.abego.treelayout.util.DefaultTreeForTreeLayout;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;

import java.awt.geom.Rectangle2D;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.function.Consumer;

public class ImGuiImNodesBehaviorTreePanel
{
   private final String windowName;

   private int nodeIndex = 0;
   private int pinIndex = 0;
   private int linkIndex = 0;
   private boolean firstRun = true;
   private final HashMap<Integer, ImGuiMovingPlot> tickPlots = new HashMap<>();

   private final HashMap<Integer, ImVec2> nodeSize = new HashMap<>();
   private final HashMap<Integer, Boolean> nodeSizeHasChanged = new HashMap<>();

   private final Class<?> parent;

   private static final String propertiesDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + ImGuiImNodesBehaviorTreePanel.class.getSimpleName() + "Settings" + File.separator;

   /***
    * Note that create() and dispose() should be called for creating and destroying the context.
    *
    * @param name The name of the window
    */
   public ImGuiImNodesBehaviorTreePanel(String name, Class<?> parent)
   {
      windowName = ImGuiTools.uniqueLabel(getClass(), name);
      this.parent = parent;
   }

   private void resetNodeIndex(GDXBehaviorUIInterface tree)
   {
      nodeIndex = tree.generateUID();
   }

   public void create()
   {
      ImNodes.createContext();

      ImNodes.pushColorStyle(ImNodesColorStyle.GridBackground, ImColor.floatToColor(0.9f, 0.9f, 0.9f));
      ImNodes.pushColorStyle(ImNodesColorStyle.NodeBackground, ImColor.floatToColor(0.8f, 0.8f, 0.8f));
      ImNodes.pushColorStyle(ImNodesColorStyle.NodeBackgroundHovered, ImColor.floatToColor(0.8f, 0.8f, 0.8f));
      ImNodes.pushColorStyle(ImNodesColorStyle.NodeBackgroundSelected, ImColor.floatToColor(0.8f, 0.8f, 0.8f));
      ImNodes.pushColorStyle(ImNodesColorStyle.Link, ImColor.floatToColor(0, 0, 0));
      ImNodes.pushColorStyle(ImNodesColorStyle.LinkHovered, ImColor.floatToColor(0, 0, 0));
      ImNodes.pushColorStyle(ImNodesColorStyle.LinkSelected, ImColor.floatToColor(0, 0, 0));
      ImNodes.pushStyleVar(ImNodesStyleVar.NodeBorderThickness, 5);
      ImNodes.pushStyleVar(ImNodesStyleVar.PinCircleRadius, 0);
   }

   public void renderImGuiWidgets(GDXBehaviorUIInterface tree)
   {
      ImGui.pushFont(ImGuiTools.getNodeFont());
      ImNodes.beginNodeEditor();
      resetNodeIndex(tree);
      ArrayList<Pair<Integer, Integer>> links = new ArrayList<>();
      renderNodeAndChildren(tree, -1, links);
      renderLinks(links);

      if (firstRun)
      {
         Path treeProperties = Paths.get(propertiesDirectory, "tree" + parent.getSimpleName());

         if (Files.exists(treeProperties)) {
            layoutNodesFromFile(treeProperties);
         } else {
            if (!applyDefaultNodeLayouts(tree)) //Returns false if no default layout exists
               layoutNodes(tree);

            new File(treeProperties.toAbsolutePath().getParent().toString()).mkdirs(); //Make necessary directories
            saveLayoutToFile(treeProperties);
         }
      }

      ImNodes.endNodeEditor();
      ImGui.popFont();
      firstRun = false;
   }

   private void layoutNodesFromFile(Path file) {
      JSONFileTools.load(file, jsonNode -> {
         Iterator<Map.Entry<String, JsonNode>> it = jsonNode.fields();
         while (it.hasNext()) {
            Map.Entry<String, JsonNode> entry = it.next();

            int id = Integer.parseInt(entry.getKey());
            String[] pos = entry.getValue().asText().split(",");
            float x = Float.parseFloat(pos[0]);
            float y = Float.parseFloat(pos[1]);

            ImNodes.setNodeGridSpacePos(id, x, y);
         }
      });
   }

   private void saveLayoutToFile(Path file) {
      Consumer<ObjectNode> rootConsumer = root ->
      {
         for (int node : nodeSize.keySet()) {
            root.put(Integer.toString(node), ImNodes.getNodeGridSpacePosX(node) + "," + ImNodes.getNodeGridSpacePosY(node));
         }
      };

      JSONFileTools.save(file, rootConsumer);
   }

   private boolean applyDefaultNodeLayouts(GDXBehaviorUIInterface tree)
   {
      return applyDefaultNodeLayouts(tree, tree);
   }

   private boolean applyDefaultNodeLayouts(GDXBehaviorUIInterface tree, GDXBehaviorUIInterface root)
   {
      boolean out = false;
      Point2D pos = tree.getTreeNodeInitialPosition();
      if (pos.getX() != 0 || pos.getY() != 0)
      {
         out = true;
         ImNodes.setNodeGridSpacePos(getIndexOfNode(tree, root), pos.getX32(), pos.getY32());
      }

      for (GDXBehaviorUIInterface child : tree.getUIChildren())
      {
         out |= applyDefaultNodeLayouts(child, root);
      }

      return out;
   }

   private void constructAbegoTree(GDXBehaviorUIInterface tree, DefaultTreeForTreeLayout<GDXBehaviorUIInterface> layout)
   {
      if (tree == null)
         return;

      for (GDXBehaviorUIInterface child : tree.getUIChildren())
      {
         layout.addChild(tree, child);
         constructAbegoTree(child, layout);
      }
   }

   private int getIndexOfNodeInternal(GDXBehaviorUIInterface node, GDXBehaviorUIInterface root)
   {
      if (root.equals(node))
         return nodeIndex;
      else
      {
         for (GDXBehaviorUIInterface child : root.getUIChildren())
         {
            nodeIndex++;
            int val = getIndexOfNodeInternal(node, child);

            if (val != -1)
               return val;
         }
      }

      return -1;
   }

   private int getIndexOfNode(GDXBehaviorUIInterface node, GDXBehaviorUIInterface tree)
   {
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
      for (GDXBehaviorUIInterface node : map.keySet())
      {
         int index = getIndexOfNode(node, tree);
         Rectangle2D.Double pos = map.get(node);

         ImNodes.setNodeGridSpacePos(index, (float) pos.x, (float) pos.y);
      }
   }

   public void renderLinks(ArrayList<Pair<Integer, Integer>> links)
   {
      for (Pair<Integer, Integer> p : links)
      {
         ImNodes.link(linkIndex++, p.getFirst(), p.getSecond());
      }
   }

   private void renderNodeAndChildren(GDXBehaviorUIInterface node, int parentPin, ArrayList<Pair<Integer, Integer>> links)
   {
      long timeSinceLastTick = -1;

      timeSinceLastTick = System.currentTimeMillis() - node.getLastTickMillis();
      boolean isTickRecent = timeSinceLastTick < 5000;

      int color;
      if (node.getPreviousStatus() == BehaviorTreeNodeStatus.SUCCESS && isTickRecent)
      {
         color = ImColor.floatToColor(0.19607843f, 0.658823529412f, 0.321568627451f);
      }
      else if (node.getPreviousStatus() == BehaviorTreeNodeStatus.FAILURE && isTickRecent)
      {
         color = ImColor.floatToColor(0.658823529412f, 0.19607843f, 0.19607843f);
      }
      else
      {
         color = ImColor.floatToColor(0.19607843f, 0.321568627451f, 0.921568627451f);
      }

      ImNodes.pushColorStyle(ImNodesColorStyle.NodeOutline, color);

      ImNodes.beginNode(nodeIndex);

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

      ImGui.textUnformatted(nodeType + " " + nodeName);

      if (parentPin != -1)
      {
         ImGui.sameLine();

         ImNodes.beginInputAttribute(pinIndex);
         ImGui.dummy(1f, 1f);
         ImNodes.endInputAttribute();

         links.add(new Pair<>(parentPin, pinIndex++));
      }

      int nodePinIndex = pinIndex;
      for (GDXBehaviorUIInterface child : node.getUIChildren())
      {
         ImGui.sameLine();

         ImNodes.beginOutputAttribute(pinIndex++);
         ImGui.dummy(1f, 1f);
         ImNodes.endOutputAttribute();
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
         tickPlot.calculate(status != null && tickedRecently ? status.name() : "", true);

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

      ImNodes.endNode();

      if (firstRun)
      {
         ImVec2 size = new ImVec2();
         ImNodes.getNodeDimensions(nodeIndex, size);

         nodeSize.put(nodeIndex, size);
         nodeSizeHasChanged.put(nodeIndex, false);
      } else {
         ImVec2 size = new ImVec2();
         ImNodes.getNodeDimensions(nodeIndex, size);
         ImVec2 correct = nodeSize.get(nodeIndex);

         if (size.x - correct.x > 0.5f || size.y - correct.y > 0.5f && !nodeSizeHasChanged.get(nodeIndex)) { //query nodeSizeHasChanged to prevent multiple warnings
            nodeSizeHasChanged.put(nodeIndex, true);
            LogTools.warn("Node size has changed for node " + nodeIndex + " (" + nodeName + ") - when implementing renderTreeNode(), ensure the node renders at a fixed size.");
         }
      }

      nodeIndex++;

      ImNodes.popColorStyle();

      for (GDXBehaviorUIInterface child : node.getUIChildren())
      {
         renderNodeAndChildren(child, nodePinIndex++, links);
      }
   }

   public void dispose()
   {
      ImNodes.destroyContext();
   }

   public String getWindowName()
   {
      return windowName;
   }
}
