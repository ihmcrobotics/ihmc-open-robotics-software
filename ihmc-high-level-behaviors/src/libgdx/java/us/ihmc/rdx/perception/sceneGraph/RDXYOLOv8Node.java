package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXYOLOv8Node extends RDXDetectableSceneNode
{
   private final YOLOv8Node yoloNode;

   private final ImGuiPlot confidencePlot;

   public RDXYOLOv8Node(YOLOv8Node yoloNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloNode);
      this.yoloNode = yoloNode;

      confidencePlot = new ImGuiPlot(labels.get("Confidence"), 1000, 230, 22);
      confidencePlot.setYScale(0.0f, 1.0f);
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      confidencePlot.setWidth((int) (0.65 * ImGui.getWindowWidth()));
      ImGui.pushStyleColor(ImGuiCol.PlotLines, ImGuiTools.greenRedGradientColor((float) yoloNode.getConfidence(), 1.0f, 0.0f));
      confidencePlot.render(yoloNode.getConfidence());
      ImGui.popStyleColor();
   }
}
