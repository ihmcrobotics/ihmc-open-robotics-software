package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPredefinedRigidBodySceneNodeBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.Set;

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
   private final RDXPanel panel = new RDXPanel("Perception Scene Graph UI", this::renderImGuiWidgets, false, true);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImBoolean viewAsTree = new ImBoolean(false);

   private final RDXPredefinedRigidBodySceneNodeBuilder predefinedRigidBodySceneNodeBuilder;

   public RDXSceneGraphUI(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, RDX3DPanel panel3D, ReferenceFrameLibrary referenceFrameLibrary)
   {
      uiRootNode = new RDXSceneNode(SceneGraph.ROOT_NODE_ID, SceneGraph.ROOT_NODE_NAME);
      sceneGraph = new ROS2SceneGraph(uiRootNode,
                                      (sceneGraph, ros2SceneGraphSubscriptionNode) -> RDXSceneGraphTools.createNodeFromMessage(ros2SceneGraphSubscriptionNode,
                                                                                                                               panel3D,
                                                                                                                               sceneGraph),
                                      ros2PublishSubscribeAPI,
                                      ROS2ActorDesignation.OPERATOR);
      referenceFrameLibrary.addDynamicCollection(sceneGraph.asNewDynamicReferenceFrameCollection());

      predefinedRigidBodySceneNodeBuilder = new RDXPredefinedRigidBodySceneNodeBuilder(sceneGraph);

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
      ImGui.beginMenuBar();

      ImGui.pushItemWidth(100.0f);
      if (ImGui.beginMenu(labels.get("Nodes")))
      {
         sceneGraph.modifyTree(modificationQueue ->
         {
            if (ImGui.beginTable("##predefinedRigidBodyTable", 2))
            {
               ImGui.tableSetupColumn(labels.get("Predefined rigid model"), ImGuiTableColumnFlags.WidthFixed, 150f);
               ImGui.tableSetupColumn(labels.get("Options"), ImGuiTableColumnFlags.WidthFixed, 200f);
               ImGui.tableHeadersRow();

               // Predefined rigid bodies
               ImGui.tableNextRow();
               ImGui.tableSetColumnIndex(0);

               if (!predefinedRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
               {
                  ImGui.beginDisabled();
               }
               if (ImGui.beginTable("##predefinedRigidBodyTableModel", 1))
               {
                  ImGui.tableNextRow();
                  ImGui.tableSetColumnIndex(0);
                  if (ImGui.button(labels.get("Add Box")))
                  {
                     modificationQueue.accept(new SceneGraphNodeAddition(predefinedRigidBodySceneNodeBuilder.build("Box"),
                                                                         predefinedRigidBodySceneNodeBuilder.getParent()));
                  }
                  if (ImGui.button(labels.get("Add Can")))
                  {
                     modificationQueue.accept(new SceneGraphNodeAddition(predefinedRigidBodySceneNodeBuilder.build("CanOfSoup"),
                                                                         predefinedRigidBodySceneNodeBuilder.getParent()));
                  }
                  if (ImGui.button(labels.get("Add 2x4")))
                  {
                     modificationQueue.accept(new SceneGraphNodeAddition(predefinedRigidBodySceneNodeBuilder.build("2X4"),
                                                                         predefinedRigidBodySceneNodeBuilder.getParent()));
                  }
                  ImGui.endTable();
               }
               if (!predefinedRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
               {
                  ImGui.endDisabled();
               }

               ImGui.tableSetColumnIndex(1);
               predefinedRigidBodySceneNodeBuilder.renderImGuiWidgets();

               ImGui.endTable();
            }
            if (!predefinedRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
            {
               ImGuiTools.previousWidgetTooltip(predefinedRigidBodySceneNodeBuilder.getRejectionTooltip());
            }
         });

         ImGui.endMenu();
      }
      ImGui.popItemWidth();

      ImGui.endMenuBar();
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

      sceneGraph.modifyTree(modificationQueue ->
      {
         if (viewAsTree.get())
         {
            renderSceneNodesAsTree(sceneGraph.getRootNode(), modificationQueue);
         }
         else // Render IDs in order so they don't jump around
         {
            for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
            {
               if (sceneNode instanceof RDXSceneNodeInterface uiSceneNode)
               {
                  ImGuiTools.textBold(sceneNode.getName());
                  uiSceneNode.renderImGuiWidgets();
                  if (sceneNode != sceneGraph.getRootNode())
                     uiSceneNode.renderRemove(modificationQueue, sceneGraph);
                  ImGui.separator();
               }
            }
         }
      });
   }

   private void renderSceneNodesAsTree(SceneNode sceneNode, SceneGraphModificationQueue modificationQueue)
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
            uiSceneNode.renderRemove(modificationQueue, sceneGraph);
            for (SceneNode child : sceneNode.getChildren())
            {
               renderSceneNodesAsTree(child, modificationQueue);
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