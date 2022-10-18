package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;

import java.nio.FloatBuffer;
import java.util.function.Function;

public class GDXPointCloudVisualizer extends ImGuiGDXVisualizer implements RenderableProvider
{
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot segmentIndexPlot = new ImGuiPlot("Segment", 1000, 230, 20);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final int pointsPerSegment;
   private final int numberOfSegments;
   private int totalNumberOfPoints;
   private final Color color = new Color();
   private final String topicName;
   private int latestSegmentIndex;

   public GDXPointCloudVisualizer(String title, String topicName, int pointsPerSegment, int numberOfSegments)
   {
      super(title + " (ROS 2)");
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      totalNumberOfPoints = pointsPerSegment * numberOfSegments;
      this.topicName = topicName;
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(pointsPerSegment, numberOfSegments);
   }

   @Override
   public void update()
   {
      super.update();
      updateMeshFastest();
   }

   public void updateMeshFastest()
   {
      pointCloudRenderer.updateMeshFastest(totalNumberOfPoints);
   }

   public void updateMeshFastest(Function<FloatBuffer, Integer> bufferConsumer)
   {
      pointCloudRenderer.updateMeshFastest(bufferConsumer);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topicName);
      ImGui.sameLine();
      ImGui.pushItemWidth(30.0f);
      ImGui.dragFloat(labels.get("Size"), pointSize.getData(), 0.001f, 0.0005f, 0.1f);
      ImGui.popItemWidth();
//      frequencyPlot.renderImGuiWidgets();
      segmentIndexPlot.render(latestSegmentIndex);
   }

   public FloatBuffer getVertexBuffer()
   {
      return pointCloudRenderer.getVertexBuffer();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   public void setLatestSegmentIndex(int latestSegmentIndex)
   {
      this.latestSegmentIndex = latestSegmentIndex;
   }

   public void setTotalNumberOfPoints(int totalNumberOfPoints)
   {
      this.totalNumberOfPoints = totalNumberOfPoints;
   }
}
