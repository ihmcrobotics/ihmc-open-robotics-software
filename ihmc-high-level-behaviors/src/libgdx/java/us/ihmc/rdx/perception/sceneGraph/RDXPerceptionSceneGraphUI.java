package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.*;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;
import java.util.Set;

/**
 * Manages perception scene graph nodes.
 * - It keeps them up to date with a subscription to the on robot perception process.
 * - It renders them in the 3D scene as semi-transparent models.
 * - TODO: It allows the operator to override the poses of nodes.
 */
public class RDXPerceptionSceneGraphUI
{
   private final PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ROS2DetectableSceneNodesSubscription detectableSceneNodesSubscription;
   private final ROS2DetectableSceneNodesPublisher detectableSceneObjectsPublisher = new ROS2DetectableSceneNodesPublisher();
   private final RDXPanel panel = new RDXPanel("Perception Scene Graph UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ArrayList<RDXPredefinedRigidBodySceneNode> predefinedRigidBodySceneNodes = new ArrayList<>();
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);

   public RDXPerceptionSceneGraphUI(PredefinedSceneNodeLibrary predefinedSceneNodeLibrary,
                                    ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                    RDX3DPanel panel3D)
   {
      this.predefinedSceneNodeLibrary = predefinedSceneNodeLibrary;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;

      detectableSceneNodesSubscription = new ROS2DetectableSceneNodesSubscription(predefinedSceneNodeLibrary.getDetectableSceneNodes(),
                                                                                  ros2PublishSubscribeAPI,
                                                                                  ROS2IOTopicQualifier.STATUS);

      for (DetectableSceneNode detectableSceneNode : predefinedSceneNodeLibrary.getDetectableSceneNodes())
      {
         if (detectableSceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
         {
            predefinedRigidBodySceneNodes.add(new RDXPredefinedRigidBodySceneNode(predefinedRigidBodySceneNode, panel3D));
         }
      }
   }

   public void update()
   {
      detectableSceneNodesSubscription.update();

      for (RDXPredefinedRigidBodySceneNode predefinedRigidBodySceneNode : predefinedRigidBodySceneNodes)
      {
         predefinedRigidBodySceneNode.update();
      }

      if (publishThrottler.run())
         detectableSceneObjectsPublisher.publish(predefinedSceneNodeLibrary.getDetectableSceneNodes(), ros2PublishSubscribeAPI, ROS2IOTopicQualifier.COMMAND);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Detectable scene nodes received: " + detectableSceneNodesSubscription.getNumberOfMessagesReceived());
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
         for (RDXPredefinedRigidBodySceneNode predefinedRigidBodySceneNode : predefinedRigidBodySceneNodes)
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