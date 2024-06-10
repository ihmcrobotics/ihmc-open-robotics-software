package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.YOLOv8.YOLOv8Node;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.List;
import java.util.Set;

public class RDXYOLOv8Node extends RDXDetectableSceneNode
{
   private final YOLOv8Node yoloNode;

   private final ImGuiUniqueLabelMap labels;

   private final ImFloat detectionAcceptanceThreshold;
   private final ImGuiPlot confidencePlot = new ImGuiPlot("Confidence", 1000, 230, 22);

   private final RDXPointCloudRenderer objectPointCloudRenderer = new RDXPointCloudRenderer();

   public RDXYOLOv8Node(YOLOv8Node yoloNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloNode);
      this.yoloNode = yoloNode;
      this.labels = labels;

      detectionAcceptanceThreshold = new ImFloat(yoloNode.getDetectionAcceptanceThreshold());

      confidencePlot.setYScale(0.0f, 1.0f);

      objectPointCloudRenderer.create(5000);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      super.update(modificationQueue);

      yoloNode.setDetectionAcceptanceThreshold(detectionAcceptanceThreshold.get());
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      confidencePlot.setWidth((int) (0.65 * ImGui.getWindowWidth()));
      ImGui.pushStyleColor(ImGuiCol.PlotLines, ImGuiTools.greenRedGradientColor((float) yoloNode.getDetection().confidence(), 1.0f, 0.0f));
      confidencePlot.render(yoloNode.getDetection().confidence());
      ImGui.popStyleColor();

      if (ImGuiTools.volatileInputFloat(labels.get("Detection Acceptance Threshold"), detectionAcceptanceThreshold))
         detectionAcceptanceThreshold.set((float) MathTools.clamp(detectionAcceptanceThreshold.get(), 0.0f, 1.0f));
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      List<Point3D32> renderablePointCloud = yoloNode.getObjectPointCloud();
      objectPointCloudRenderer.setPointsToRender(renderablePointCloud, Color.GREEN);
      objectPointCloudRenderer.updateMesh();
      objectPointCloudRenderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      objectPointCloudRenderer.dispose();
   }
}
