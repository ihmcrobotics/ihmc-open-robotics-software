package us.ihmc.rdx.perception.sceneGraph;

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
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;

import java.util.ArrayList;
import java.util.Set;

/**
 * Updates and renders perception scene graph nodes.
 */
public class RDXPerceptionSceneGraphUI
{
   private final PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private final ROS2DetectableSceneNodesSubscription detectableSceneNodesSubscription;
   private final ImGuiPanel panel = new ImGuiPanel("Perception Scene Graph UI", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ArrayList<RDXPredefinedRigidBodySceneNode> predefinedRigidBodySceneNodes = new ArrayList<>();
   private final ArrayList<ImPlotDoublePlot> detectableNodeCurrentlyDetectedPlots = new ArrayList<>();

   public RDXPerceptionSceneGraphUI(PredefinedSceneNodeLibrary predefinedSceneNodeLibrary, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.predefinedSceneNodeLibrary = predefinedSceneNodeLibrary;

      detectableSceneNodesSubscription = new ROS2DetectableSceneNodesSubscription(predefinedSceneNodeLibrary.getDetectableSceneNodes(),
                                                                                  ros2PublishSubscribeAPI);

      for (DetectableSceneNode detectableSceneNode : predefinedSceneNodeLibrary.getDetectableSceneNodes())
      {
         if (detectableSceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
         {
            RDXPredefinedRigidBodySceneNode rdxPredefinedRigidBodySceneNode = new RDXPredefinedRigidBodySceneNode(predefinedRigidBodySceneNode);
            predefinedRigidBodySceneNodes.add(rdxPredefinedRigidBodySceneNode);
         }

         int plotHeightInPixels = 35;
         detectableNodeCurrentlyDetectedPlots.add(new ImPlotDoublePlot(detectableSceneNode.getName(), plotHeightInPixels));
      }
   }

   public void update()
   {
      detectableSceneNodesSubscription.update();

      for (RDXPredefinedRigidBodySceneNode predefinedRigidBodySceneNode : predefinedRigidBodySceneNodes)
      {
         predefinedRigidBodySceneNode.update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Detectable scene nodes received: " + detectableSceneNodesSubscription.getNumberOfMessagesReceived());
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
      ImGui.text("Detections:");
      for (int i = 0; i < detectableNodeCurrentlyDetectedPlots.size(); i++)
      {
         ImPlotDoublePlot detectableNodeCurrentlyDetectedPlot = detectableNodeCurrentlyDetectedPlots.get(i);
         detectableNodeCurrentlyDetectedPlot.addValue(predefinedSceneNodeLibrary.getDetectableSceneNodes().get(i).getCurrentlyDetected() ? 1.0 : 0.0);
         detectableNodeCurrentlyDetectedPlot.renderImGuiWidgets();
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

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}