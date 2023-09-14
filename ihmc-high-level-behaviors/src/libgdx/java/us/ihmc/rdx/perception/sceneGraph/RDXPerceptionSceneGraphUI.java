package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneGraphNodeMove;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphPublisher;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscription;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.tools.thread.Throttler;

import java.util.*;

/**
 * Manages perception scene graph nodes.
 * - It keeps them up to date with a subscription to the on robot perception process.
 * - It renders them in the 3D scene as semi-transparent models.
 * - TODO: It allows the operator to override the poses of nodes.
 */
public class RDXPerceptionSceneGraphUI
{
   private final SceneGraph sceneGraph;
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final RDXSceneNode uiRootNode;
   private final ROS2SceneGraphSubscription sceneGraphSubscription;
   private final ROS2SceneGraphPublisher sceneGraphPublisher = new ROS2SceneGraphPublisher();
   private final RDXPanel panel = new RDXPanel("Perception Scene Graph UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImBoolean viewAsTree = new ImBoolean(false);
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);
   private final SortedSet<SceneNode> sceneNodesByID = new TreeSet<>(Comparator.comparingLong(SceneNode::getID));

   public RDXPerceptionSceneGraphUI(SceneGraph sceneGraph,
                                    ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                    RDX3DPanel panel3D)
   {
      this.sceneGraph = sceneGraph;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;

      if (sceneGraph.getRootNode() instanceof RDXSceneNode rootNode)
      {
         uiRootNode = rootNode;
      }
      else
      {
         throw new RuntimeException("Scene graph must be created with RDXSceneGraph::new");
      }

      sceneGraphSubscription = new ROS2SceneGraphSubscription
      (
         sceneGraph,
         ros2PublishSubscribeAPI,
         ROS2IOTopicQualifier.STATUS,
         ros2SceneGraphSubscriptionNode -> RDXSceneGraphTools.createNodeFromMessage(ros2SceneGraphSubscriptionNode, panel3D, sceneGraph)
      );
      sceneGraphSubscription.getSceneGraphSubscription().addCallback(message -> frequencyPlot.recordEvent());
      frequencyPlot.getReceivedPlot().setWidth(100);
   }

   public void update()
   {
      sceneGraphSubscription.update();

      // Nodes can be moved by the user clicking stuff
      sceneGraph.getSceneGraphNodeMoves().clear();
      sceneNodesByID.clear();

      update(uiRootNode);

      for (SceneGraphNodeMove sceneGraphNodeMove : sceneGraph.getSceneGraphNodeMoves())
      {
         sceneGraphNodeMove.performMove();
      }

      if (publishThrottler.run())
         sceneGraphPublisher.publish(sceneGraph, ros2PublishSubscribeAPI, ROS2IOTopicQualifier.COMMAND);
   }

   private void update(RDXSceneNodeInterface uiSceneNode)
   {
      uiSceneNode.update(sceneGraph.getSceneGraphNodeMoves());

      if (uiSceneNode instanceof SceneNode sceneNode)
      {
         sceneNodesByID.add(sceneNode);
      }

      if (uiSceneNode instanceof SceneNode sceneNode)
      {
         for (SceneNode child : sceneNode.getChildren())
         {
            if (child instanceof RDXSceneNodeInterface uiChildNode)
            {
               update(uiChildNode);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      int numberOfLocalNodes = sceneGraph.getIDToNodeMap().size();
      int numberOfOnRobotNodes = sceneGraphSubscription.getLatestSceneGraphMessage().getSceneTreeIndices().size();
      ImGui.text("UI nodes: %d   On robot nodes: %d   State: ".formatted(numberOfLocalNodes, numberOfOnRobotNodes));
      ImGui.sameLine();
      if (sceneGraphSubscription.getLocalTreeFrozen())
         ImGui.textColored(ImGuiTools.LIGHT_BLUE, "Frozen");
      else
         ImGui.text("Normal");
      ImGui.pushItemWidth(100.0f);
      frequencyPlot.renderImGuiWidgets();
      ImGui.popItemWidth();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("View as tree"), viewAsTree);
      ImGui.separator();

      if (viewAsTree.get())
      {
         renderSceneNodesAsTree(sceneGraph.getRootNode());
      }
      else // Render IDs in order so they don't jump around
      {
         for (SceneNode sceneNode : sceneNodesByID)
         {
            if (sceneNode instanceof RDXSceneNodeInterface uiSceneNode)
            {
               ImGuiTools.textBold(sceneNode.getName());
               uiSceneNode.renderImGuiWidgets();
               ImGui.separator();
            }
         }
      }
   }

   private void renderSceneNodesAsTree(SceneNode sceneNode)
   {
      if (sceneNode instanceof RDXSceneNodeInterface uiSceneNode)
      {
         float indentReduction = 10.0f; // Less indent to take less space
         ImGui.unindent(indentReduction);

         boolean expanded = false;
         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         if (ImGui.treeNode(labels.get(sceneNode.getName())))
         {
            expanded = true;
            ImGui.popFont();


            uiSceneNode.renderImGuiWidgets();
            for (SceneNode child : sceneNode.getChildren())
            {
               renderSceneNodesAsTree(child);
            }
            ImGui.treePop();
         }

         if (!expanded)
            ImGui.popFont();

         ImGui.indent(indentReduction);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         getRenderables(uiRootNode, renderables, pool, sceneLevels);
      }
   }

   private void getRenderables(RDXSceneNodeInterface uiSceneNode, Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      uiSceneNode.getRenderables(renderables, pool, sceneLevels);

      if (uiSceneNode instanceof SceneNode sceneNode)
      {
         for (SceneNode child : sceneNode.getChildren())
         {
            if (child instanceof RDXSceneNodeInterface uiChildNode)
            {
               getRenderables(uiChildNode, renderables, pool, sceneLevels);
            }
         }
      }
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}