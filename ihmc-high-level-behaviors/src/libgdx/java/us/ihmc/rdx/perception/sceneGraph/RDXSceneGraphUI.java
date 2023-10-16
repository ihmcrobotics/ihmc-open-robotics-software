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
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPredefinedRigidBodySceneNodeBuilder;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPrimitiveRigidBodySceneNodeBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Manages the perception scene graph.
 * - It keeps them up to date with a subscription to the on robot perception process.
 * - It renders them in the 3D scene as semi-transparent models.
 * - Allows the operator to override the poses of nodes.
 */
public class RDXSceneGraphUI
{
   private final ROS2SceneGraph sceneGraph;
   private final RDXPanel panel = new RDXPanel("Perception Scene Graph UI", this::renderImGuiWidgets, false, true);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImBoolean viewAsTree = new ImBoolean(false);
   private final Map<SceneNode, RDXSceneNode> uiSceneNodes = new ConcurrentHashMap<>();

   private final RDXPredefinedRigidBodySceneNodeBuilder predefinedRigidBodySceneNodeBuilder;
   private final RDXPrimitiveRigidBodySceneNodeBuilder primitiveRigidBodySceneNodeBuilder;

   public RDXSceneGraphUI(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, RDX3DPanel panel3D, ReferenceFrameLibrary referenceFrameLibrary)
   {
      sceneGraph = new ROS2SceneGraph(new SceneNode(SceneGraph.ROOT_NODE_ID, SceneGraph.ROOT_NODE_NAME), (sceneGraph, ros2SceneGraphSubscriptionNode) ->
      {
         RDXSceneNode uiSceneNode = RDXSceneGraphTools.createNodeFromMessage(ros2SceneGraphSubscriptionNode, panel3D, sceneGraph);
         uiSceneNodes.put(uiSceneNode.getSceneNode(), uiSceneNode);
         return uiSceneNode.getSceneNode();
      }, ros2PublishSubscribeAPI, ROS2ActorDesignation.OPERATOR);

      uiSceneNodes.put(sceneGraph.getRootNode(), new RDXSceneNode(sceneGraph.getRootNode()));

      sceneGraph.getSceneGraphSubscription().getSceneGraphSubscription().addCallback(message -> subscriptionFrequencyText.ping());

      predefinedRigidBodySceneNodeBuilder = new RDXPredefinedRigidBodySceneNodeBuilder(sceneGraph);
      primitiveRigidBodySceneNodeBuilder = new RDXPrimitiveRigidBodySceneNodeBuilder(sceneGraph);
   }

   public void update()
   {
      sceneGraph.updateSubscription();
      sceneGraph.modifyTree(modificationQueue -> uiSceneNodes.values().forEach(node ->
                                                                               {
                                                                                  node.update(modificationQueue);
                                                                               }));
      sceneGraph.updatePublication();

      for (SceneNode sceneNode : uiSceneNodes.keySet())
         if (!sceneGraph.getSceneNodesByID().contains(sceneNode))
            uiSceneNodes.remove(sceneNode);
   }

   private void renderMenuBar()
   {
      if (ImGui.beginMenuBar())
      {
         ImGui.pushItemWidth(100.0f);
         if (ImGui.beginMenu(labels.get("Nodes")))
         {
            sceneGraph.modifyTree(modificationQueue ->
            {
               // Predefined rigid bodies
               if (ImGui.beginTable("##predefinedRigidBodyTable", 2))
               {
                  ImGui.tableSetupColumn(labels.get("Predefined rigid model"), ImGuiTableColumnFlags.WidthFixed, 150f);
                  ImGui.tableSetupColumn(labels.get("Options"), ImGuiTableColumnFlags.WidthFixed, 200f);
                  ImGui.tableHeadersRow();

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
                        RDXPredefinedRigidBodySceneNode box = predefinedRigidBodySceneNodeBuilder.build("Box");

                        uiSceneNodes.put(box.getSceneNode(), box);

                        modificationQueue.accept(new SceneGraphNodeAddition(box.getSceneNode(),
                                                                            predefinedRigidBodySceneNodeBuilder.getParent()));
                     }
                     if (ImGui.button(labels.get("Add Can")))
                     {
                        RDXPredefinedRigidBodySceneNode canOfSoup = predefinedRigidBodySceneNodeBuilder.build("CanOfSoup");

                        uiSceneNodes.put(canOfSoup.getSceneNode(), canOfSoup);

                        modificationQueue.accept(new SceneGraphNodeAddition(canOfSoup.getSceneNode(),
                                                                            predefinedRigidBodySceneNodeBuilder.getParent()));
                     }
                     if (ImGui.button(labels.get("Add 2x4")))
                     {
                        RDXPredefinedRigidBodySceneNode twoByFour = predefinedRigidBodySceneNodeBuilder.build("2X4");

                        uiSceneNodes.put(twoByFour.getSceneNode(), twoByFour);

                        modificationQueue.accept(new SceneGraphNodeAddition(twoByFour.getSceneNode(),
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

               // Primitive rigid bodies
               if (ImGui.beginTable("##primitiveRigidBodyTable", 2))
               {
                  ImGui.tableSetupColumn(labels.get("Primitive shape"), ImGuiTableColumnFlags.WidthFixed, 150f);
                  ImGui.tableSetupColumn(labels.get("Options"), ImGuiTableColumnFlags.WidthFixed, 200f);
                  ImGui.tableHeadersRow();

                  ImGui.tableNextRow();
                  ImGui.tableSetColumnIndex(0);

                  if (!primitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
                  {
                     ImGui.beginDisabled();
                  }
                  if (ImGui.beginTable("##primitiveRigidBodyTableModel", 1))
                  {
                     ImGui.tableNextRow();
                     ImGui.tableSetColumnIndex(0);
                     for (PrimitiveRigidBodyShape shape : PrimitiveRigidBodyShape.values())
                     {
                        if (ImGui.button(labels.get("Add " + shape.toString().toLowerCase())))
                        {
                           RDXPrimitiveRigidBodySceneNode rdxPrimitiveRigidBodySceneNode = primitiveRigidBodySceneNodeBuilder.build(shape);

                           uiSceneNodes.put(rdxPrimitiveRigidBodySceneNode.getSceneNode(), rdxPrimitiveRigidBodySceneNode);

                           modificationQueue.accept(new SceneGraphNodeAddition(rdxPrimitiveRigidBodySceneNode.getSceneNode(),
                                                                               primitiveRigidBodySceneNodeBuilder.getParent()));
                        }
                     }
                     ImGui.endTable();
                  }
                  if (!primitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
                  {
                     ImGui.endDisabled();
                  }

                  ImGui.tableSetColumnIndex(1);
                  primitiveRigidBodySceneNodeBuilder.renderImGuiWidgets();

                  ImGui.endTable();
               }
               if (!primitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
               {
                  ImGuiTools.previousWidgetTooltip(primitiveRigidBodySceneNodeBuilder.getRejectionTooltip());
               }
            });

            ImGui.endMenu();
         }
         ImGui.popItemWidth();
      }

      ImGui.endMenuBar();
   }

   public void renderImGuiWidgets()
   {
      renderMenuBar();

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
            renderSceneNodesAsTree(modificationQueue, sceneGraph.getRootNode());
         }
         else // Render IDs in order so they don't jump around
         {
            for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
            {
               if (uiSceneNodes.containsKey(sceneNode))
               {
                  ImGuiTools.textBold(sceneNode.getName());
                  uiSceneNodes.get(sceneNode).renderImGuiWidgets(modificationQueue, sceneGraph);
                  ImGui.separator();
               }
            }
         }
      });
   }

   private void renderSceneNodesAsTree(SceneGraphModificationQueue modificationQueue, SceneNode sceneNode)
   {
      if (uiSceneNodes.containsKey(sceneNode))
      {
         RDXSceneNode uiSceneNode = uiSceneNodes.get(sceneNode);

         float indentReduction = 10.0f; // Less indent to take less space
         ImGui.unindent(indentReduction);

         boolean expanded = false;
         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         if (ImGui.treeNode("##sceneNode-" + sceneNode.getID(), sceneNode.getName()))
         {
            expanded = true;
            ImGui.popFont();

            uiSceneNode.renderImGuiWidgets(modificationQueue, sceneGraph);
            for (SceneNode child : sceneNode.getChildren())
            {
               renderSceneNodesAsTree(modificationQueue, child);
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
         uiSceneNodes.values().forEach(node -> node.getRenderables(renderables, pool, sceneLevels));
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