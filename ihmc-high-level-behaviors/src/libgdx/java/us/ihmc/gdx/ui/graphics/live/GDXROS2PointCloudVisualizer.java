package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.internal.ImGui;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class GDXROS2PointCloudVisualizer extends ImGuiGDXVisualizer implements RenderableProvider
{
   private static final int MAX_POINTS = 500000;

   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private final ResettableExceptionHandlingExecutorService threadQueue;

   private GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);

   private Point3D32[] points;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   public GDXROS2PointCloudVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
   }

   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      ++receivedCount;
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            points = StereoPointCloudCompression.decompressPointCloudToArray32(message);
            //         int[] colors = PointCloudCompression.decompressColorsToIntArray(message);
         });
      }
   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      ++receivedCount;
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            int numberOfPoints = message.getNumberOfPoints();
            Point3D32[] points = new Point3D32[numberOfPoints];
//            int[] colors = new int[numberOfPoints];

            LidarPointCloudCompression.decompressPointCloud(message.getScan(), numberOfPoints, (i, x, y, z) ->
            {
               points[i] = new Point3D32();
               points[i].setX(x);
               points[i].setY(y);
               points[i].setZ(z);
            });

            this.points = points;
         });
      }
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(MAX_POINTS);
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive() && points != null)
      {
         pointsToRender.clear();
         for (Point3D32 point : points)
         {
            pointsToRender.add().set(point);
         }

         pointCloudRenderer.setPointsToRender(pointsToRender);
         pointCloudRenderer.updateMesh();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      receivedPlot.render(receivedCount);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }
}
