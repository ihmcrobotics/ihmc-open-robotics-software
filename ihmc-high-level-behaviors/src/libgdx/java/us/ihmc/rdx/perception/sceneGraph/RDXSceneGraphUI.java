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
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiTreeRenderer;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPredefinedRigidBodySceneNodeBuilder;
import us.ihmc.rdx.perception.sceneGraph.builder.RDXPrimitiveRigidBodySceneNodeBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

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
   private final RDX3DPanel panel3D;
   private final RDXPanel panel = new RDXPanel("Scene Graph", this::renderImGuiWidgets, false, true);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImBoolean viewAsTree = new ImBoolean(false);
   private final Map<SceneNode, RDXSceneNode> uiSceneNodes = new ConcurrentHashMap<>(); // Use addUISceneNode() and removeUISceneNode() to modify
   private final ImGuiTreeRenderer treeRenderer = new ImGuiTreeRenderer();

   private final RDXPredefinedRigidBodySceneNodeBuilder predefinedRigidBodySceneNodeBuilder;
   private final RDXPrimitiveRigidBodySceneNodeBuilder primitiveRigidBodySceneNodeBuilder;
   private final RDXPrimitiveRigidBodySceneNodeBuilder predefinedPrimitiveRigidBodySceneNodeBuilder;

   public RDXSceneGraphUI(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, RDX3DPanel panel3D)
   {
      sceneGraph = new ROS2SceneGraph(new SceneNode(SceneGraph.ROOT_NODE_ID, SceneGraph.ROOT_NODE_NAME), (sceneGraph, ros2SceneGraphSubscriptionNode) ->
      {
         RDXSceneNode uiSceneNode = RDXSceneGraphTools.createNodeFromMessage(ros2SceneGraphSubscriptionNode, panel3D, sceneGraph);
         addUISceneNode(uiSceneNode);
         return uiSceneNode.getSceneNode();
      }, ros2PublishSubscribeAPI, ROS2ActorDesignation.OPERATOR);

      this.panel3D = panel3D;

      RDXSceneNode rootNode = new RDXSceneNode(sceneGraph.getRootNode());
      addUISceneNode(rootNode);

      sceneGraph.getSceneGraphSubscription().getSceneGraphSubscription().addCallback(message -> subscriptionFrequencyText.ping());

      predefinedRigidBodySceneNodeBuilder = new RDXPredefinedRigidBodySceneNodeBuilder(sceneGraph);
      primitiveRigidBodySceneNodeBuilder = new RDXPrimitiveRigidBodySceneNodeBuilder(sceneGraph);
      predefinedPrimitiveRigidBodySceneNodeBuilder = new RDXPrimitiveRigidBodySceneNodeBuilder(sceneGraph);
   }

   private void addUISceneNode(RDXSceneNode uiSceneNode)
   {
      uiSceneNodes.put(uiSceneNode.getSceneNode(), uiSceneNode);

      panel3D.getNotificationManager().pushNotification("Added SceneNode [" + uiSceneNode.getSceneNode().getName() + "]");
   }

   public void update()
   {
      sceneGraph.updateSubscription();
      sceneGraph.modifyTree(modificationQueue -> uiSceneNodes.values().forEach(node -> node.update(modificationQueue)));
      sceneGraph.updatePublication();

      for (SceneNode sceneNode : uiSceneNodes.keySet())
      {
         RDXSceneNode uiNode = uiSceneNodes.get(sceneNode);

         // If the node was removed from the UI or the node doesn't exist in the scene graph at all
         if (uiNode.isRemoved() || !sceneGraph.getSceneNodesByID().contains(sceneNode))
         {
            // Destroy and remove UI scene node
            uiNode.destroy();
            uiSceneNodes.remove(sceneNode);

            // Remove the scene node from the scene graph
            sceneGraph.modifyTree(modificationQueue -> {
               modificationQueue.accept(new SceneGraphClearSubtree(sceneNode));
               modificationQueue.accept(new SceneGraphNodeRemoval(sceneNode, sceneGraph));
            });

            // Send a notification
            panel3D.getNotificationManager().pushNotification("Removed SceneNode [" + sceneNode.getName() + "]");
         }
      }
   }

   private void renderMenuBar(SceneGraphModificationQueue modificationQueue)
   {
      if (ImGui.beginMenuBar())
      {
         ImGui.pushItemWidth(100.0f);
         if (ImGui.beginMenu(labels.get("Nodes")))
         {
            // Predefined rigid bodies
            if (ImGui.beginTable("##predefinedRigidBodyTable", 2))
            {
               ImGui.tableSetupColumn(labels.get("Predefined rigid model"), ImGuiTableColumnFlags.WidthFixed, 150f);
               ImGui.tableSetupColumn(labels.get("Options"), ImGuiTableColumnFlags.WidthFixed, 200f);
               ImGui.tableHeadersRow();

               ImGui.tableNextRow();
               ImGui.tableSetColumnIndex(0);

               ImGui.beginDisabled(!predefinedRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty());
               if (ImGui.beginTable("##predefinedRigidBodyTableModel", 1))
               {
                  ImGui.tableNextRow();
                  ImGui.tableSetColumnIndex(0);
                  if (ImGui.button(labels.get("Add Box")))
                  {
                     RDXPredefinedRigidBodySceneNode box = predefinedRigidBodySceneNodeBuilder.build("Box");
                     modificationQueue.accept(new SceneGraphNodeAddition(box.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(box);
                  }
                  if (ImGui.button(labels.get("Add Can")))
                  {
                     RDXPredefinedRigidBodySceneNode canOfSoup = predefinedRigidBodySceneNodeBuilder.build("CanOfSoup");
                     modificationQueue.accept(new SceneGraphNodeAddition(canOfSoup.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(canOfSoup);
                  }
                  if (ImGui.button(labels.get("Add 2x4")))
                  {
                     RDXPredefinedRigidBodySceneNode twoByFour = predefinedRigidBodySceneNodeBuilder.build("2X4");
                     modificationQueue.accept(new SceneGraphNodeAddition(twoByFour.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(twoByFour);
                  }
                  if (ImGui.button(labels.get("Add Work Platform")))
                  {
                     RDXPredefinedRigidBodySceneNode workPlatform = predefinedRigidBodySceneNodeBuilder.build("WorkPlatform");
                     modificationQueue.accept(new SceneGraphNodeAddition(workPlatform.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(workPlatform);
                  }
                  if (ImGui.button(labels.get("Add Shoe")))
                  {
                     RDXPredefinedRigidBodySceneNode shoe = predefinedRigidBodySceneNodeBuilder.build("Shoe");
                     modificationQueue.accept(new SceneGraphNodeAddition(shoe.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(shoe);
                  }
                  if (ImGui.button(labels.get("Add Laptop")))
                  {
                     RDXPredefinedRigidBodySceneNode laptop = predefinedRigidBodySceneNodeBuilder.build("Laptop");
                     modificationQueue.accept(new SceneGraphNodeAddition(laptop.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(laptop);
                  }
                  if (ImGui.button(labels.get("Add Book")))
                  {
                     RDXPredefinedRigidBodySceneNode book = predefinedRigidBodySceneNodeBuilder.build("Book");
                     modificationQueue.accept(new SceneGraphNodeAddition(book.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(book);
                  }
                  if (ImGui.button(labels.get("Add Cereal")))
                  {
                     RDXPredefinedRigidBodySceneNode cereal = predefinedRigidBodySceneNodeBuilder.build("Cereal");
                     modificationQueue.accept(new SceneGraphNodeAddition(cereal.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(cereal);
                  }
                  if (ImGui.button(labels.get("Add Mug")))
                  {
                     RDXPredefinedRigidBodySceneNode mug = predefinedRigidBodySceneNodeBuilder.build("Mug");
                     modificationQueue.accept(new SceneGraphNodeAddition(mug.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(mug);
                  }
                  if (ImGui.button(labels.get("Add Bike")))
                  {
                     RDXPredefinedRigidBodySceneNode bike = predefinedRigidBodySceneNodeBuilder.build("Bike");
                     modificationQueue.accept(new SceneGraphNodeAddition(bike.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(bike);
                  }
                  if (ImGui.button(labels.get("Add Drill")))
                  {
                     RDXPredefinedRigidBodySceneNode drill = predefinedRigidBodySceneNodeBuilder.build("Drill");
                     modificationQueue.accept(new SceneGraphNodeAddition(drill.getSceneNode(), predefinedRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(drill);
                  }
                  ImGui.endTable();
               }
               ImGui.endDisabled();


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

               ImGui.beginDisabled(!primitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty());
               if (ImGui.beginTable("##primitiveRigidBodyTableModel", 1))
               {
                  ImGui.tableNextRow();
                  ImGui.tableSetColumnIndex(0);
                  for (PrimitiveRigidBodyShape shape : PrimitiveRigidBodyShape.values())
                  {
                     if (ImGui.button(labels.get("Add " + shape.toString().toLowerCase())))
                     {
                        RDXPrimitiveRigidBodySceneNode primitiveRigidBodySceneNode = primitiveRigidBodySceneNodeBuilder.build(shape);
                        modificationQueue.accept(new SceneGraphNodeAddition(primitiveRigidBodySceneNode.getSceneNode(), primitiveRigidBodySceneNodeBuilder.getParent()));
                        addUISceneNode(primitiveRigidBodySceneNode);
                     }
                  }
                  ImGui.endTable();
               }
               ImGui.endDisabled();

               ImGui.tableSetColumnIndex(1);
               primitiveRigidBodySceneNodeBuilder.renderImGuiWidgets();

               ImGui.endTable();
            }
            if (!primitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
            {
               ImGuiTools.previousWidgetTooltip(primitiveRigidBodySceneNodeBuilder.getRejectionTooltip());
            }

            // Predefined primitives
            if (ImGui.beginTable("##predefinedPrimitiveRigidBodyTable", 2))
            {
               ImGui.tableSetupColumn(labels.get("Predefined primitive"), ImGuiTableColumnFlags.WidthFixed, 150f);
               ImGui.tableSetupColumn(labels.get("Options"), ImGuiTableColumnFlags.WidthFixed, 200f);
               ImGui.tableHeadersRow();

               ImGui.tableNextRow();
               ImGui.tableSetColumnIndex(0);

               ImGui.beginDisabled(!predefinedPrimitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty());
               if (ImGui.beginTable("##primitiveRigidBodyTableModel", 1))
               {
                  ImGui.tableNextRow();
                  ImGui.tableSetColumnIndex(0);

                  if (ImGui.button(labels.get("Add small box")))
                  {
                     Vector3D32 smallBoxLengths = new Vector3D32(0.206f, 0.254f, 0.165f);
                     RDXPrimitiveRigidBodySceneNode smallBox = predefinedPrimitiveRigidBodySceneNodeBuilder.build(PrimitiveRigidBodyShape.BOX, smallBoxLengths, null);
                     modificationQueue.accept(new SceneGraphNodeAddition(smallBox.getSceneNode(), predefinedPrimitiveRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(smallBox);
                     smallBox.place();
                  }
                  if (ImGui.button(labels.get("Add medium box")))
                  {
                     Vector3D32 mediumBoxLengths = new Vector3D32(0.3f, 0.4f, 0.2f);
                     RDXPrimitiveRigidBodySceneNode mediumBox = predefinedPrimitiveRigidBodySceneNodeBuilder.build(PrimitiveRigidBodyShape.BOX, mediumBoxLengths, null);
                     modificationQueue.accept(new SceneGraphNodeAddition(mediumBox.getSceneNode(), predefinedPrimitiveRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(mediumBox);
                     mediumBox.place();
                  }
                  if (ImGui.button(labels.get("Add large box")))
                  {
                     Vector3D32 largeBoxLengths = new Vector3D32(0.48f, 0.48f, 0.49f);
                     RDXPrimitiveRigidBodySceneNode largeBox = predefinedPrimitiveRigidBodySceneNodeBuilder.build(PrimitiveRigidBodyShape.BOX, largeBoxLengths, null);
                     modificationQueue.accept(new SceneGraphNodeAddition(largeBox.getSceneNode(), predefinedPrimitiveRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(largeBox);
                     largeBox.place();
                  }
                  if (ImGui.button(labels.get("Add open box")))
                  {
                     Vector3D32 largeBoxLengths = new Vector3D32(0.314f, 0.394f, 0.26f);
                     RDXPrimitiveRigidBodySceneNode largeBox = predefinedPrimitiveRigidBodySceneNodeBuilder.build(PrimitiveRigidBodyShape.BOX, largeBoxLengths, null);
                     modificationQueue.accept(new SceneGraphNodeAddition(largeBox.getSceneNode(), predefinedPrimitiveRigidBodySceneNodeBuilder.getParent()));
                     addUISceneNode(largeBox);
                     largeBox.place();
                  }
                  ImGui.endTable();
               }

               ImGui.endDisabled();

               ImGui.tableSetColumnIndex(1);
               predefinedPrimitiveRigidBodySceneNodeBuilder.renderImGuiWidgets();

               ImGui.endTable();
            }
            if (!predefinedPrimitiveRigidBodySceneNodeBuilder.getRejectionTooltip().isEmpty())
            {
               ImGuiTools.previousWidgetTooltip(predefinedPrimitiveRigidBodySceneNodeBuilder.getRejectionTooltip());
            }

            ImGui.endMenu();
         }
         ImGui.popItemWidth();
      }

      ImGui.endMenuBar();
   }

   public void renderImGuiWidgets()
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         renderMenuBar(modificationQueue);

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
            renderSceneNodesAsTree(modificationQueue, sceneGraph.getRootNode());
         }
         else // Render IDs in order so they don't jump around
         {
            for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
            {

               if (uiSceneNodes.containsKey(sceneNode))
               {
                  if (ImGui.collapsingHeader(labels.get(sceneNode.getName())))
                  {
                     uiSceneNodes.get(sceneNode).renderImGuiWidgets(modificationQueue, sceneGraph);
                  }
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

         treeRenderer.render(sceneNode.getID(), sceneNode.getName(), () ->
         {
            uiSceneNode.renderImGuiWidgets(modificationQueue, sceneGraph);
            for (SceneNode child : sceneNode.getChildren())
            {
               renderSceneNodesAsTree(modificationQueue, child);
            }
         });
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         uiSceneNodes.values().forEach(node ->
         {
            if (!node.isGraphicsHidden())
               node.getRenderables(renderables, pool, sceneLevels);
         });
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