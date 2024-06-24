package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.rdx.imgui.ImGuiEnumPlot;
import us.ihmc.rdx.imgui.ImGuiTools;

public abstract class RDXDetectableSceneNode extends RDXSceneNode
{
   private final DetectableSceneNode detectableSceneNode;
   private final ImGuiEnumPlot currentlyDetectedPlot;

   public RDXDetectableSceneNode(DetectableSceneNode detectableSceneNode)
   {
      super(detectableSceneNode);

      this.detectableSceneNode = detectableSceneNode;
      currentlyDetectedPlot = new ImGuiEnumPlot();
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      super.update(sceneGraph);

      detectableSceneNode.update(sceneGraph);
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);
      ImGui.sameLine();

      boolean currentlyDetected = detectableSceneNode.getCurrentlyDetected();
      currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
      currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");
   }
}
