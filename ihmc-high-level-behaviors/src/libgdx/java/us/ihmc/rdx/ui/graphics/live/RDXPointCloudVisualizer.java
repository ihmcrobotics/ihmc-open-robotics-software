package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;

import java.nio.FloatBuffer;
import java.util.function.Function;

/**
 * RDXPointCloudVisualizer is intended to visualize any pointCloud data through GDX and ImGui api.
 * <p>
 * It expects to receive datatype: {@link PointCloud}
 * </p>
 **/

public class RDXPointCloudVisualizer extends RDXVisualizer
{
   // Note: Renderer. Creates and sets up shader, generate mesh and provides renderable.
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();

   // NOTE: Variables for ImGuiWidget render.
   private final String topicName;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private int latestSegmentIndex;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot segmentIndexPlot = new ImGuiPlot("Segment", 1000, 230, 20);
   private final ImFloat pointSize = new ImFloat(0.01f);

   // NOTE: Some parameters of data
   private final int pointsPerSegment;
   private final int numberOfSegments;
   private int totalNumberOfPoints;
   private final int numberOfElementsPerPoint;

   public RDXPointCloudVisualizer(String title, String topicName, int pointsPerSegment, int numberOfSegments, int numberOfElementsPerPoint)
   {
      super(title + " (ROS 2)");
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      totalNumberOfPoints = pointsPerSegment * numberOfSegments;
      this.topicName = topicName;
      this.numberOfElementsPerPoint = numberOfElementsPerPoint;
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(pointsPerSegment, numberOfSegments);
      pointCloudRenderer.getVertexBuffer().limit(pointsPerSegment * numberOfElementsPerPoint);
   }

   @Override
   public void update()
   {
      super.update();
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
      frequencyPlot.renderImGuiWidgets();
      segmentIndexPlot.render(latestSegmentIndex);
   }

   public FloatBuffer getVertexBuffer()
   {
      return pointCloudRenderer.getVertexBuffer();
   }

   public void updatePointCloud(PointCloud pointCloud)
   {
      getVertexBuffer().put(pointCloud.getData());
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

   public void recordEventFrequency(boolean messageQueued)
   {
      if (messageQueued)
         frequencyPlot.recordEvent();
   }
}
