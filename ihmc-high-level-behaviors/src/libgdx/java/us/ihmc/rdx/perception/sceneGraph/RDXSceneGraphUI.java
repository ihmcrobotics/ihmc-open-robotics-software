package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.*;

/**
 * Manages the perception scene graph.
 * - It keeps them up to date with a subscription to the on robot perception process.
 * - It renders them in the 3D scene as semi-transparent models.
 * - Allows the operator to override the poses of nodes.
 */
public class RDXSceneGraphUI
{
   private final ROS2SceneGraph sceneGraph;
   private final RDXSceneNode uiRootNode;
   private final RDXPanel panel = new RDXPanel("Perception Scene Graph UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImBoolean viewAsTree = new ImBoolean(false);

   public RDXSceneGraphUI(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, RDX3DPanel panel3D)
   {
      uiRootNode = new RDXSceneNode(SceneGraph.ROOT_NODE_ID, SceneGraph.ROOT_NODE_NAME);
      sceneGraph = new ROS2SceneGraph
      (
         uiRootNode,
         (sceneGraph, ros2SceneGraphSubscriptionNode) -> RDXSceneGraphTools.createNodeFromMessage(ros2SceneGraphSubscriptionNode, panel3D, sceneGraph),
         ros2PublishSubscribeAPI,
         ROS2ActorDesignation.OPERATOR
      );

      sceneGraph.getSceneGraphSubscription().getSceneGraphSubscription().addCallback(message -> subscriptionFrequencyText.ping());
   }

   public void update()
   {
      sceneGraph.updateSubscription();
      sceneGraph.modifyTree(modificationQueue -> update(uiRootNode, modificationQueue));
      sceneGraph.updatePublication();
   }

   private void update(RDXSceneNodeInterface uiSceneNode, SceneGraphModificationQueue modificationQueue)
   {
      uiSceneNode.update(modificationQueue);

      if (uiSceneNode instanceof SceneNode sceneNode)
      {
         for (SceneNode child : sceneNode.getChildren())
         {
            if (child instanceof RDXSceneNodeInterface uiChildNode)
            {
               update(uiChildNode, modificationQueue);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      int numberOfLocalNodes = sceneGraph.getIDToNodeMap().size();
      SceneGraphMessage latestSceneGraphMessage = sceneGraph.getSceneGraphSubscription().getLatestSceneGraphMessage();
      int numberOfOnRobotNodes = latestSceneGraphMessage == null ? 0 : latestSceneGraphMessage.getSceneTreeIndices().size();
      ImGui.text("UI nodes: %d   On robot nodes: %d   State: ".formatted(numberOfLocalNodes, numberOfOnRobotNodes));
      ImGui.sameLine();
      if (sceneGraph.getSceneGraphSubscription().getLocalTreeFrozen())
         ImGui.textColored(ImGuiTools.LIGHT_BLUE, "Frozen");
      else
         ImGui.text("Normal");
      ImGui.pushItemWidth(100.0f);
      subscriptionFrequencyText.render();
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
         for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
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
         if (ImGui.treeNode("##sceneNode-" + sceneNode.getID(), sceneNode.getName()))
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

   public ROS2SceneGraph getSceneGraph()
   {
      return sceneGraph;
   }
}