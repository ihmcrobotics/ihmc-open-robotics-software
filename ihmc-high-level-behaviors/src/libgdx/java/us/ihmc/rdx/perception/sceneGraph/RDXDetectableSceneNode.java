package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
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
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.sameLine();

      boolean currentlyDetected = detectableSceneNode.getCurrentlyDetected();
      currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
      currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");
   }
}
