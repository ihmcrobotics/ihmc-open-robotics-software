package us.ihmc.rdx.perception.scene;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.ROS2DetectableSceneNodesSubscription;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.scene.objects.RDXPredefinedRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.ArrayList;
import java.util.Set;

/**
 * Updates and renders perception scene graph nodes.
 */
public class RDXPerceptionSceneUI
{
   private final PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private final ROS2DetectableSceneNodesSubscription detectableSceneNodesSubscription;
   private final ImGuiPanel panel = new ImGuiPanel("Objects Perception UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ArrayList<RDXPredefinedRigidBodySceneNode> sceneObjects = new ArrayList<>();

   public RDXPerceptionSceneUI(PredefinedSceneNodeLibrary predefinedSceneNodeLibrary, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.predefinedSceneNodeLibrary = predefinedSceneNodeLibrary;

      detectableSceneNodesSubscription = new ROS2DetectableSceneNodesSubscription(predefinedSceneNodeLibrary.getDetectableSceneNodes(),
                                                                                  ros2PublishSubscribeAPI);

      for (DetectableSceneNode detectableSceneNode : predefinedSceneNodeLibrary.getDetectableSceneNodes())
      {
         if (detectableSceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
         {
            RDXPredefinedRigidBodySceneNode rdxPredefinedRigidBodySceneNode = new RDXPredefinedRigidBodySceneNode(predefinedRigidBodySceneNode);
            sceneObjects.add(rdxPredefinedRigidBodySceneNode);
         }
      }
   }

   public void update()
   {
      detectableSceneNodesSubscription.update();

      for (RDXPredefinedRigidBodySceneNode sceneObject : sceneObjects)
      {
         sceneObject.update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         for (RDXPredefinedRigidBodySceneNode sceneObject : sceneObjects)
         {
            sceneObject.getRenderables(renderables, pool, sceneLevels);
         }
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}