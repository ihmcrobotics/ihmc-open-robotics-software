package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.rdx.imgui.ImGuiEnumPlot;
import us.ihmc.rdx.imgui.ImGuiTools;

public class RDXDetectableSceneNode extends RDXSceneNode
{
   private final DetectableSceneNode detectableSceneNode;
   private final ImGuiEnumPlot currentlyDetectedPlot;

   public RDXDetectableSceneNode(DetectableSceneNode detectableSceneNode)
   {
      super(detectableSceneNode);
      this.detectableSceneNode = detectableSceneNode;
      currentlyDetectedPlot = new ImGuiEnumPlot(detectableSceneNode.getName());
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      boolean currentlyDetected = detectableSceneNode.getCurrentlyDetected();
      currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
      currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");
   }
}
