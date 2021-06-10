package us.ihmc.gdx.ui.behaviors;

import imgui.extension.nodeditor.NodeEditor;
import imgui.extension.nodeditor.NodeEditorContext;
import imgui.extension.nodeditor.flag.NodeEditorPinKind;
import imgui.extension.nodeditor.flag.NodeEditorStyleColor;
import imgui.extension.nodeditor.flag.NodeEditorStyleVar;
import imgui.internal.ImGui;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

import java.util.ArrayList;
import java.util.HashMap;

public class ImGuiBehaviorTreePanel
{
   private final String windowName;

   private int nodeIndex = 0;
   private int pinIndex = 0;
   private int linkIndex = 0;
   private boolean firstRun = true;
   private final HashMap<Integer, ImGuiMovingPlot> tickPlots = new HashMap<>();

   private NodeEditorContext context = null;

   /***
    * Note that create() and dispose() should be called for creating and destroying the context.
    *
    * @param name The name of the window
    */
   public ImGuiBehaviorTreePanel(String name)
   {
      windowName = ImGuiTools.uniqueLabel(getClass(), name);
   }

   public void create() {
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
      NodeEditor.begin(windowName);
      nodeIndex = 1;
      ArrayList<Pair<Integer, Integer>> links = new ArrayList<>();
      renderNodeAndChildren(tree, -1, links);
      renderLinks(links);

      NodeEditor.end();
      firstRun = false;
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
      else if (node.getType().equals(BehaviorTreeAction.class))
      {
         ImGui.textUnformatted("[Action] " + (name != null ? name : "Action"));
      }
      else if (node.getType().equals(BehaviorTreeCondition.class))
      {
         ImGui.textUnformatted("[Condition] " + (name != null ? name : "Condition"));
      }
      else if (node.getType().equals(OneShotAction.class))
      {
         ImGui.textUnformatted("[OneShot] " + (name != null ? name : "One Shot Action"));
      }
      else if (node.getType().equals(AlwaysSuccessfulAction.class))
      {
         ImGui.textUnformatted("[Success] " + (name != null ? name : "Always Successful Action"));
      }
      else
      {
         ImGui.textUnformatted("[*] " + (name != null ? name : "Behavior " + nodeIndex));
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
         tickPlot.render(status != null && tickedRecently ? status.name() : "");
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
      }

      node.renderTreeNode();

      if (firstRun)
      {
         Point2D position = node.getTreeNodeInitialPosition();
         NodeEditor.setNodePosition(nodeIndex, position.getX32(), position.getY32());
      }

      NodeEditor.endNode();
      nodeIndex++;

      NodeEditor.popStyleColor(1);

      for (GDXBehaviorUIInterface child : node.getUIChildren())
      {
         renderNodeAndChildren(child, nodePinIndex++, links);
      }
   }

   public void dispose() {
      NodeEditor.destroyEditor(context);
   }
}
