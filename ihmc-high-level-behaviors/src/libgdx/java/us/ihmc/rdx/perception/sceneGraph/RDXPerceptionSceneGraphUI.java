package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.*;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphPublisher;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscription;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.tools.thread.Throttler;

import java.util.Set;

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
   private final ROS2SceneGraphSubscription sceneGraphSubscription;
   private final ROS2SceneGraphPublisher sceneGraphPublisher = new ROS2SceneGraphPublisher();
   private final RDXPanel panel = new RDXPanel("Perception Scene Graph UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private RDXSceneNode rootNode;
   private final TLongObjectMap<RDXSceneNode> idsToNodesMap = new TLongObjectHashMap<>();
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);

   public RDXPerceptionSceneGraphUI(SceneGraph sceneGraph,
                                    ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                    RDX3DPanel panel3D)
   {
      this.sceneGraph = sceneGraph;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;

      sceneGraphSubscription = new ROS2SceneGraphSubscription(sceneGraph, ros2PublishSubscribeAPI, ROS2IOTopicQualifier.STATUS);

      for (DetectableSceneNode detectableSceneNode : sceneGraph.getDetectableSceneNodes())
      {
         if (detectableSceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
         {
            predefinedRigidBodySceneNodes.add(new RDXSceneNode(predefinedRigidBodySceneNode, panel3D));
         }
      }
   }

   public void update()
   {
      sceneGraphSubscription.update();





      for (RDXSceneNode predefinedRigidBodySceneNode : predefinedRigidBodySceneNodes)
      {
         predefinedRigidBodySceneNode.update();
      }

      if (publishThrottler.run())
         sceneGraphPublisher.publish(sceneGraph, ros2PublishSubscribeAPI, ROS2IOTopicQualifier.COMMAND);
   }

   private void clearTree(RDXSceneNode node)
   {
      for (RDXSceneNode child : node.getChildren())
      {
         clearTree(child);
      }

      node.getChildren().clear();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Scene graph updates received: " + sceneGraphSubscription.getNumberOfMessagesReceived());
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
      ImGui.text("Detections:");
      ImGui.separator();
      for (int i = 0; i < predefinedRigidBodySceneNodes.size(); i++)
      {
         predefinedRigidBodySceneNodes.get(i).renderImGuiWidgets();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         for (RDXSceneNode predefinedRigidBodySceneNode : predefinedRigidBodySceneNodes)
         {
            predefinedRigidBodySceneNode.getRenderables(renderables, pool, sceneLevels);
         }
      }
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}