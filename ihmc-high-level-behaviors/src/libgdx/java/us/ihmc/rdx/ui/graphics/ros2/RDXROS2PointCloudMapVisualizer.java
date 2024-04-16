package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.elements.DiscretizedColoredPointCloud;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLIntBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXROS2PointCloudMapVisualizer extends RDXVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot segmentIndexPlot = new ImGuiPlot("Segment", 1000, 230, 20);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<RDXPointCloudRenderer> pointCloudRenderers = new ArrayList<>();
   private final int pointsPerSegment;
   private final int totalNumberOfPoints;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
   private final ByteBuffer decompressionInputDirectBuffer;
   private final int inputBytesPerPoint = 4 * Integer.BYTES;
   private final AtomicReference<FusedSensorHeadPointCloudMessage> latestFusedSensorHeadPointCloudMessageReference = new AtomicReference<>(null);
   private final AtomicReference<LidarScanMessage> latestLidarScanMessageReference = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionMessageReference = new AtomicReference<>(null);
   private final Color color = new Color();
   private int latestSegmentIndex = -1;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLIntBuffer decompressedOpenCLIntBuffer;
   private OpenCLFloatBuffer parametersOpenCLFloatBuffer;
   private ROS2SyncedRobotModel syncedRobot;
   private Vector3D previousTranslation = null;
   private RotationMatrix previousRotation = null;

   public RDXROS2PointCloudMapVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> topic, ROS2SyncedRobotModel syncedRobot)
   {
      this(title, ros2Node, topic, syncedRobot, 500000);
   }

   public RDXROS2PointCloudMapVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> topic, ROS2SyncedRobotModel syncedRobot, int pointsPerSegment)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.syncedRobot = syncedRobot;
      this.pointsPerSegment = pointsPerSegment;
      totalNumberOfPoints = pointsPerSegment;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      decompressionInputDirectBuffer = ByteBuffer.allocateDirect(pointsPerSegment * inputBytesPerPoint);
      decompressionInputDirectBuffer.order(ByteOrder.nativeOrder());

      if (topic.getType().equals(LidarScanMessage.class))
      {
         new ROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new ROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
      else if (topic.getType().equals(FusedSensorHeadPointCloudMessage.class))
      {
         new ROS2Callback<>(ros2Node,
                                topic.withType(FusedSensorHeadPointCloudMessage.class),
                                this::queueRenderFusedSensorHeadPointCloud);
      }
   }

   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      frequencyPlot.recordEvent();
      // TODO: Possibly decompress on a thread here
      // TODO: threadQueue.clearQueueAndExecute(() ->
      latestStereoVisionMessageReference.set(message);
   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      frequencyPlot.recordEvent();
      latestLidarScanMessageReference.set(message);
   }

   private void queueRenderFusedSensorHeadPointCloud(FusedSensorHeadPointCloudMessage message)
   {
      frequencyPlot.recordEvent();
      latestFusedSensorHeadPointCloudMessageReference.set(message);
   }

   public RDXPointCloudRenderer addPointCloudRenderer()
   {
      pointCloudRenderers.add(new RDXPointCloudRenderer());
      pointCloudRenderers.get(pointCloudRenderers.size() - 1).create(pointsPerSegment);

      return pointCloudRenderers.get(pointCloudRenderers.size() - 1);
   }

   public RDXPointCloudRenderer getLatestRenderer()
   {
      return pointCloudRenderers.get(pointCloudRenderers.size() - 1);
   }

   public boolean isRobotMoved()
   {
      RigidBodyTransform currentTransform = syncedRobot.getReferenceFrames().getOusterLidarFrame().getTransformToWorldFrame();
      Vector3D currentTranslation = (Vector3D) currentTransform.getTranslation();
      RotationMatrix currentRotation = (RotationMatrix) currentTransform.getRotation();

      if (previousTranslation == null)
      {
         previousTranslation = currentTranslation;
         previousRotation = currentRotation;
         return true;
      }

      double distTraveled = previousTranslation.differenceNorm(currentTranslation);
      double distRotated = previousRotation.distance(currentRotation);
      previousTranslation = currentTranslation;
      previousRotation = currentRotation;

      if (distRotated > Math.PI / 60 || distTraveled > 0.1)
      {
         return true;
      }

      return false;
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLProgram = openCLManager.loadProgram("FusedSensorPointCloudSubscriberVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "unpackPointCloud");

      parametersOpenCLFloatBuffer = new OpenCLFloatBuffer(4);
      parametersOpenCLFloatBuffer.createOpenCLBufferObject(openCLManager);
      decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointsPerSegment * 4);
      decompressedOpenCLIntBuffer.createOpenCLBufferObject(openCLManager);
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         FusedSensorHeadPointCloudMessage fusedMessage = latestFusedSensorHeadPointCloudMessageReference.getAndSet(null);
         if (fusedMessage != null && isRobotMoved())
         {
            decompressionInputDirectBuffer.rewind();
            int numberOfBytes = fusedMessage.getScan().size();
            decompressionInputDirectBuffer.limit(numberOfBytes);
            for (int i = 0; i < numberOfBytes; i++)
            {
               decompressionInputDirectBuffer.put(fusedMessage.getScan().get(i));
            }
            decompressionInputDirectBuffer.flip();
            decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
            // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
            lz4Decompressor.decompress(decompressionInputDirectBuffer, decompressedOpenCLIntBuffer.getBackingDirectByteBuffer());
            decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();

            latestSegmentIndex = (int) fusedMessage.getSegmentIndex();

            parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, latestSegmentIndex);
            parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, pointSize.get());
            parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);
            parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(3, (float) DiscretizedColoredPointCloud.DISCRETE_RESOLUTION);

            parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
            decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);

            openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());

            RDXPointCloudRenderer latestRenderer = addPointCloudRenderer();
            pointCloudVertexBuffer = new OpenCLFloatBuffer(pointsPerSegment * 8, latestRenderer.getVertexBuffer());
            pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);

            openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
            openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
            pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

            latestRenderer.updateMeshFastest(totalNumberOfPoints);
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      ImGui.sameLine();
      ImGui.pushItemWidth(30.0f);
      ImGui.dragFloat(labels.get("Size"), pointSize.getData(), 0.001f, 0.0005f, 0.1f);
      ImGui.popItemWidth();
      frequencyPlot.renderImGuiWidgets();
      segmentIndexPlot.render(latestSegmentIndex);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         for (RDXPointCloudRenderer pointCloudRenderer : pointCloudRenderers)
         {
            pointCloudRenderer.getRenderables(renderables, pool);
         }
      }
   }
}
