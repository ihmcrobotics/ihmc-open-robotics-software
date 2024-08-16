package us.ihmc.rdx.simulation.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4Factory;
import org.apache.commons.lang3.mutable.MutableInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import perception_msgs.msg.dds.BigVideoPacket;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.elements.DiscretizedColoredPointCloud;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLIntBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.types.PointType;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

import static us.ihmc.rdx.simulation.sensors.RDXLowLevelDepthSensorSimulator.FLOATS_PER_POINT;

public class RDXHighLevelDepthSensorSimulator extends RDXPanel
{
   private static final MutableInt INDEX = new MutableInt();

   private Mat depthImageMat;
   private final BytePointer compressedColorPointer = new BytePointer();
   private final BytePointer compressedDepthPointer = new BytePointer();;

   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final String sensorName;
   private final FramePose3D sensorPose = new FramePose3D();
   private final ReferenceFrame sensorFrame;
   private final Matrix4 gdxTransform = new Matrix4();
   private final RDXLowLevelDepthSensorSimulator depthSensorSimulator;
   private final LongSupplier timestampSupplier;
   private final CameraPinholeBrown depthCameraIntrinsics;
   private final int imageWidth;
   private final int imageHeight;
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final Mat rgba8Mat;

   int depthSequenceNumber = 0;
   int colorSequenceNumber = 0;

   private RosNodeInterface ros1Node;
   private String ros1DepthImageTopic;
   private String ros1ColorImageTopic;
   private ROS2Topic<?> ros2PointCloudTopic;
   private ROS2Topic<ImageMessage> ros2DepthTopic;
   private ROS2Topic<ImageMessage> ros2ColorTopic;
   private String ros1PointCloudTopic;
   private RosImagePublisher ros1DepthPublisher;
   private RosPointCloudPublisher ros1PointCloudPublisher;
   private RosCameraInfoPublisher ros1DepthCameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;
   private RosImagePublisher ros1ColorPublisher;
   private RosCameraInfoPublisher ros1ColorCameraInfoPublisher;
   private ChannelBuffer ros1ColorChannelBuffer;
   private Mat rgb8Mat;
   private ByteBuffer rgb8Buffer;

   private ROS2NodeInterface ros2Node;
   private ROS2Helper ros2Helper;
   private Class<?> pointCloudMessageType;
   private ROS2PublisherBasics<?> publisher;
   private RealtimeROS2Node realtimeROS2Node;
   private ROS2PublisherBasics<BigVideoPacket> ros2VideoPublisher;
   private BigVideoPacket videoPacket;
   private BytePointer jpegImageBytePointer;
   private Mat yuv420Image;
   private IntPointer compressionParameters;
   private RecyclingArrayList<Point3D> ros1PointsToPublish;
   private RecyclingArrayList<Point3D> ros2PointsToPublish;
   private int[] ros2ColorsToPublish;
   private final FramePose3D tempSensorFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private final Throttler throttler = new Throttler();
   private final Throttler segmentedThrottler = new Throttler();
   private final ResettableExceptionHandlingExecutorService depthExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService pointCloudExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorROS2Executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ImDouble publishRateHz = new ImDouble();
   private final ImBoolean debugCoordinateFrame = new ImBoolean(false);
   private ModelInstance coordinateFrame;
   private RigidBodyTransform sensorFrameToWorldTransform;

   private boolean tuning = false;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean sensorEnabled = new ImBoolean(false);
   private final ImBoolean renderPointCloudDirectly = new ImBoolean(false);
   private final ImBoolean publishDepthImageROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS1 = new ImBoolean(false);
   private final ImBoolean publishPointCloudROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS2 = new ImBoolean(false);
   private final ImBoolean publishPointCloudROS2 = new ImBoolean(false);
   private final ImBoolean publishDepthImageMessageROS2 = new ImBoolean(false);
   private final ImBoolean publishColorImageMessageROS2 = new ImBoolean(false);

   private final ImBoolean useSensorColor = new ImBoolean(false);
   private final ImBoolean colorBasedOnWorldZ = new ImBoolean(true);
   private final Color pointColorFromPicker = new Color();
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final float[] color = new float[] {1.0f, 1.0f, 1.0f, 1.0f};
   private final ImInt segmentationDivisor = new ImInt(8);
   private final List<Point3D> pointCloud = new ArrayList<>();
   private int segmentIndex = 0;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel discretizePointsKernel;
   private OpenCLFloatBuffer worldPointCloudBuffer;
   private OpenCLFloatBuffer parametersBuffer;
   private ByteBuffer compressedPointCloudBuffer;
   private OpenCLIntBuffer discretizedIntBuffer;
   private FusedSensorHeadPointCloudMessage outputFusedROS2Message;
   private LZ4Compressor lz4Compressor;

   public RDXHighLevelDepthSensorSimulator(String sensorName,
                                           ReferenceFrame sensorFrame,
                                           LongSupplier timestampSupplier,
                                           double verticalFOV,
                                           int imageWidth,
                                           int imageHeight,
                                           double minRange,
                                           double maxRange,
                                           double noiseAmplitudeAtMinRange,
                                           double noiseAmplitudeAtMaxRange,
                                           boolean simulateL515Noise,
                                           double publishRateHz)
   {
      super(ImGuiTools.uniqueLabel(INDEX.getAndIncrement(), sensorName + " Simulator"));
      this.sensorName = sensorName;
      setRenderMethod(this::renderImGuiWidgets);
      depthSensorSimulator = new RDXLowLevelDepthSensorSimulator(sensorName,
                                                                 verticalFOV,
                                                                 imageWidth,
                                                                 imageHeight,
                                                                 minRange,
                                                                 maxRange,
                                                                 noiseAmplitudeAtMinRange,
                                                                 noiseAmplitudeAtMaxRange,
                                                                 simulateL515Noise);

      this.sensorFrame = sensorFrame;
      this.timestampSupplier = timestampSupplier;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.publishRateHz.set(publishRateHz);

      pointCloudRenderer.create(imageWidth * imageHeight);

      depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
      addChild(depthSensorSimulator.getDepthPanel());
      addChild(depthSensorSimulator.getColorPanel());

      if (debugCoordinateFrame.get())
         coordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.2);

      depthCameraIntrinsics = new CameraPinholeBrown();
      updateCameraPinholeBrown();

      rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new BytePointer(depthSensorSimulator.getColorRGBA8Buffer()));
      rgb8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3, new BytePointer(depthSensorSimulator.getColorRGBA8Buffer()));
   }

   public void setupForROS1Depth(RosNodeInterface ros1Node, String ros1DepthImageTopic, String ros1DepthCameraInfoTopic)
   {
      this.ros1Node = ros1Node;
      this.ros1DepthImageTopic = ros1DepthImageTopic;
      LogTools.info("Publishing ROS 1 depth: {} {}", ros1DepthImageTopic, ros1DepthCameraInfoTopic);
      ros1DepthPublisher = new RosImagePublisher();
      ros1DepthCameraInfoPublisher = new RosCameraInfoPublisher();
      ros1Node.attachPublisher(ros1DepthCameraInfoTopic, ros1DepthCameraInfoPublisher);
      ros1Node.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
      ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * imageWidth * imageHeight);
   }

   public void setupForROS1Color(RosNodeInterface ros1Node, String ros1ColorImageTopic, String ros1ColorCameraInfoTopic)
   {
      this.ros1Node = ros1Node;
      this.ros1ColorImageTopic = ros1ColorImageTopic;
      LogTools.info("Publishing ROS 1 color: {} {}", ros1ColorImageTopic, ros1ColorCameraInfoTopic);
      ros1ColorPublisher = new RosImagePublisher();
      ros1ColorCameraInfoPublisher = new RosCameraInfoPublisher();
      ros1Node.attachPublisher(ros1ColorCameraInfoTopic, ros1ColorCameraInfoPublisher);
      ros1Node.attachPublisher(ros1ColorImageTopic, ros1ColorPublisher);
      ros1ColorChannelBuffer = ros1ColorPublisher.getChannelBufferFactory().getBuffer(3 * imageWidth * imageHeight);
      rgb8Buffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 3);
      rgb8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3, new BytePointer(rgb8Buffer));
   }

   public void setupForROS1PointCloud(RosNodeInterface ros1Node, String ros1PointCloudTopic)
   {
      this.ros1Node = ros1Node;
      this.ros1PointCloudTopic = ros1PointCloudTopic;
      ros1PointsToPublish = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D::new);
      ros1PointCloudPublisher = new RosPointCloudPublisher(PointType.XYZ, false);
      ros1Node.attachPublisher(ros1PointCloudTopic, ros1PointCloudPublisher);
   }

   public void setupForROS2ImageMessages(ROS2NodeInterface ros2Node, ROS2Topic<ImageMessage> ros2DepthTopic, ROS2Topic<ImageMessage> ros2ColorTopic)
   {
      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);
      this.ros2DepthTopic = ros2DepthTopic;
      this.ros2ColorTopic = ros2ColorTopic;

      this.depthImageMat = new Mat();
      this.yuv420Image = new Mat();
   }

   public void setupForROS2PointCloud(ROS2NodeInterface ros2Node, ROS2Topic<?> ros2PointCloudTopic)
   {
      this.ros2Node = ros2Node;
      this.ros2PointCloudTopic = ros2PointCloudTopic;
      ros2PointsToPublish = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D::new);
      pointCloudMessageType = ros2PointCloudTopic.getType();
      if (pointCloudMessageType.equals(StereoVisionPointCloudMessage.class))
      {
         ros2ColorsToPublish = new int[imageWidth * imageHeight];
      }
      else if (pointCloudMessageType.equals(FusedSensorHeadPointCloudMessage.class))
      {
         openCLManager = new OpenCLManager();
         openCLProgram = openCLManager.loadProgram("HighLevelDepthSensorSimulator", "PerceptionCommon.cl");
         discretizePointsKernel = openCLManager.createKernel(openCLProgram, "discretizePoints");
         parametersBuffer = new OpenCLFloatBuffer(5);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
         int numberOfSegments = segmentationDivisor.get();
         int pointsPerSegment = depthSensorSimulator.getNumberOfPoints() / numberOfSegments;
         worldPointCloudBuffer = new OpenCLFloatBuffer(depthSensorSimulator.getNumberOfPoints() * 8, depthSensorSimulator.getPointCloudBuffer());
         worldPointCloudBuffer.createOpenCLBufferObject(openCLManager);
         discretizedIntBuffer = new OpenCLIntBuffer(pointsPerSegment * DiscretizedColoredPointCloud.DISCRETE_INTS_PER_POINT);
         discretizedIntBuffer.createOpenCLBufferObject(openCLManager);
         compressedPointCloudBuffer = ByteBuffer.allocateDirect(pointsPerSegment * DiscretizedColoredPointCloud.DISCRETE_BYTES_PER_POINT);
         compressedPointCloudBuffer.order(ByteOrder.nativeOrder());
         outputFusedROS2Message = new FusedSensorHeadPointCloudMessage();
         lz4Compressor = LZ4Factory.nativeInstance().fastCompressor();
      }

      LogTools.info("Publishing ROS 2 point cloud: {}", ros2PointCloudTopic.getName());
      publisher = ros2Node.createPublisher(ros2PointCloudTopic);
   }

   public void setupForROS2Color(PubSubImplementation pubSubImplementation, ROS2Topic<BigVideoPacket> ros2VideoTopic)
   {
      // A Realtime ROS 2 node is required for video streaming in order to get stable performance.
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(sensorName) + "_video");
      ros2VideoPublisher = realtimeROS2Node.createPublisher(ros2VideoTopic);
      realtimeROS2Node.spin();
      videoPacket = new BigVideoPacket();
      yuv420Image = new Mat();
      jpegImageBytePointer = new BytePointer();
      compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);
   }

   public void render(RDX3DScene scene)
   {
      if (sensorEnabled.get())
      {
         if (sensorFrame != null)
            LibGDXTools.toLibGDX(sensorFrame.getTransformToWorldFrame(), gdxTransform);
         else
            LibGDXTools.toLibGDX(sensorFrameToWorldTransform, gdxTransform);

         depthSensorSimulator.setCameraWorldTransform(gdxTransform);

         if (coordinateFrame != null)
            coordinateFrame.transform.set(gdxTransform);

         if (renderPointCloudDirectly.get())
         {
            LibGDXTools.toLibGDX(color, pointColorFromPicker);
            Color pointColor = useSensorColor.get() ? null : pointColorFromPicker;
            depthSensorSimulator.render(scene, colorBasedOnWorldZ.get(), pointColor, pointSize.get());
            pointCloudRenderer.updateMeshFastest(imageWidth * imageHeight);
         }
         else
         {
            depthSensorSimulator.render(scene);
         }
         double publishPeriod = Conversions.hertzToSeconds(publishRateHz.get());
         if (throttler.run(publishPeriod))
         {
            if (ros1Node != null && ros1Node.isStarted())
            {
               if (publishDepthImageROS1.get())
                  publishDepthImageROS1();
               if (publishColorImageROS1.get())
                  publishColorImageROS1();
               if (publishPointCloudROS1.get())
                  publishPointCloudROS1();
            }

            if (realtimeROS2Node != null)
            {
               if (publishColorImageROS2.get())
                  publishColorImageROS2();
            }

            if (ros2Node != null)
            {
               if (publishDepthImageMessageROS2.get())
                  publishROS2DepthImageMessage();
               if (publishColorImageMessageROS2.get())
                  publishROS2ColorImageMessage();
            }
         }
         if (pointCloudMessageType != null && pointCloudMessageType.equals(FusedSensorHeadPointCloudMessage.class))
            publishPeriod /= segmentationDivisor.get();
         if (segmentedThrottler.run(publishPeriod)) // This one needs to potentially run faster to publish the segments
         {
            if (ros2Node != null)
            {
               if (publishPointCloudROS2.get())
                  publishPointCloudROS2();
            }
         }

         tuning = false;
      }
   }

   private void publishColorImageROS1()
   {
      if (ros1ColorPublisher.isConnected() && ros1ColorCameraInfoPublisher.isConnected() && !colorExecutor.isExecuting())
      {
         opencv_imgproc.cvtColor(rgba8Mat, rgb8Mat, opencv_imgproc.COLOR_RGBA2RGB);

         ros1ColorChannelBuffer.clear();
         rgb8Buffer.rewind();
         ros1ColorChannelBuffer.writeBytes(rgb8Buffer);

         colorExecutor.execute(() ->
         {
            Image message = ros1ColorPublisher.createMessage(imageWidth, imageHeight, 3, "rgb8", ros1ColorChannelBuffer);

            if (timestampSupplier != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));

            ros1ColorPublisher.publish(message);
         });
      }
   }

   private void publishColorImageROS2()
   {
      if (!colorROS2Executor.isExecuting())
      {
         colorROS2Executor.execute(() ->
         {
            long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
            Instant now = Instant.now();
            videoPacket.setAcquisitionTimeSecondsSinceEpoch(now.getEpochSecond());
            videoPacket.setAcquisitionTimeAdditionalNanos(now.getNano());

            opencv_imgproc.cvtColor(rgba8Mat, yuv420Image, opencv_imgproc.COLOR_RGBA2YUV_I420);
            opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);

            byte[] heapByteArrayData = new byte[jpegImageBytePointer.asBuffer().remaining()];
            jpegImageBytePointer.asBuffer().get(heapByteArrayData);
            videoPacket.getData().resetQuick();
            videoPacket.getData().add(heapByteArrayData);
            ros2VideoPublisher.publish(videoPacket);
         });
      }
   }

   private void publishDepthImageROS1()
   {
      if (ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected() && !depthExecutor.isExecuting())
      {
         PerspectiveCamera camera = depthSensorSimulator.getCamera();
         ByteBuffer depthFloatBuffer = depthSensorSimulator.getMetersDepthFloatBuffer();
         depthFloatBuffer.rewind();
         ros1DepthChannelBuffer.clear();
         int size = 2 * imageWidth * imageHeight;
         for (int y = 0; y < imageHeight; y++)
         {
            for (int x = 0; x < imageWidth; x++)
            {
               float eyeDepthMeters = depthFloatBuffer.getFloat();

               int row = y + 1;
               int backForY = row * imageWidth * 2;
               int forwardForX = x * 2;
               int index = size - backForY + forwardForX;
               if (eyeDepthMeters > camera.near && eyeDepthMeters < depthSensorSimulator.getMaxRange())
               {
                  char depthChar16 = (char) Math.round(eyeDepthMeters * 1000.0f); // 1000 is 1 meter
                  ros1DepthChannelBuffer.setChar(index, depthChar16);
               }
               else
               {
                  ros1DepthChannelBuffer.setChar(index, 0);
               }
            }
         }

         ros1DepthChannelBuffer.readerIndex(0);
         ros1DepthChannelBuffer.writerIndex(size);

         depthExecutor.execute(() ->
         {
            if (tuning)
               updateCameraPinholeBrown();
            ros1DepthCameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time());
            Image message = ros1DepthPublisher.createMessage(imageWidth, imageHeight, 2, "16UC1", ros1DepthChannelBuffer); // maybe need to copy here if there are errors

            if(timestampSupplier != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));

            ros1DepthPublisher.publish(message);
         });
      }
   }

   private void updateCameraPinholeBrown()
   {
      depthCameraIntrinsics.setFx(depthSensorSimulator.getFocalLengthPixels().get());
      depthCameraIntrinsics.setFy(depthSensorSimulator.getFocalLengthPixels().get());
      depthCameraIntrinsics.setSkew(0.0);
      depthCameraIntrinsics.setCx(depthSensorSimulator.getPrincipalOffsetXPixels().get());
      depthCameraIntrinsics.setCy(depthSensorSimulator.getPrincipalOffsetYPixels().get());
   }

   public void renderImGuiWidgets()
   {
      tuning = true;
      ImGui.text("Resolution: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(labels.get("Sensor Enabled"), sensorEnabled);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show frame graphic"), debugCoordinateFrame);
      ImGui.text("Render:");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Point cloud"), renderPointCloudDirectly);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Depth video"), getLowLevelSimulator().getDepthPanel().getIsShowing());
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Color video"), getLowLevelSimulator().getColorPanel().getIsShowing());
      boolean publishing = ros1DepthImageTopic != null;
      publishing |= ros1ColorImageTopic != null;
      publishing |= ros1PointCloudTopic != null;
      publishing |= ros2PointCloudTopic != null;
      publishing |= realtimeROS2Node != null;
      if (publishing)
      {
         ImGui.text("Publish:");
         ImGui.sameLine();
         ImGui.text("(" + publishRateHz.get() + " Hz)");
         if (ros1DepthImageTopic != null)
            ImGui.checkbox(labels.get("ROS 1 Depth image (" + ros1DepthImageTopic + ")"), publishDepthImageROS1);
         if (ros1ColorImageTopic != null)
            ImGui.checkbox(labels.get("ROS 1 Color image (" + ros1ColorImageTopic + ")"), publishColorImageROS1);
         if (ros1PointCloudTopic != null)
            ImGui.checkbox(labels.get("ROS 1 Point Cloud (" + ros1PointCloudTopic + ")"), publishPointCloudROS1);
         if (ros2PointCloudTopic != null)
         {
            ImGui.checkbox(labels.get("ROS 2 Point cloud (" + ros2PointCloudTopic + ")"), publishPointCloudROS2);
            if (pointCloudMessageType.equals(FusedSensorHeadPointCloudMessage.class))
            {
               ImGui.text("Number of segments: " + segmentationDivisor.get());
            }
         }
         if (ros2DepthTopic != null)
         {
            ImGui.checkbox(labels.get("ROS 2 Depth image (" + ros2DepthTopic + ")"), publishDepthImageMessageROS2);
         }
         if (ros2ColorTopic != null)
         {
            ImGui.checkbox(labels.get("ROS 2 Color image (" + ros2ColorTopic + ")"), publishColorImageMessageROS2);
         }
         if (realtimeROS2Node != null)
         {
            ImGui.sameLine();
            ImGui.checkbox(labels.get("Color image (ROS 2)"), publishColorImageROS2);
         }
      }
      ImGui.checkbox("Use Sensor Color", useSensorColor);
      ImGui.sameLine();
      ImGui.checkbox("Color based on world Z", colorBasedOnWorldZ);
      ImGui.sliderFloat("Point size", pointSize.getData(), 0.0001f, 0.10f);
      if (ImGui.collapsingHeader(labels.get("Color tuner")))
      {
         ImGui.colorPicker4("Color", color);
      }
      if (ImGui.collapsingHeader(labels.get("Low level settings")))
      {
         depthSensorSimulator.renderTuningSliders();
      }
   }

   private void publishPointCloudROS2()
   {
      if (!pointCloudExecutor.isExecuting())
      {
         if (pointCloudMessageType.equals(LidarScanMessage.class) || pointCloudMessageType.equals(StereoVisionPointCloudMessage.class))
         {
            ros2PointsToPublish.clear();

            for (int i = 0; i < depthSensorSimulator.getNumberOfPoints()
                            && (FLOATS_PER_POINT * i + 2) < depthSensorSimulator.getPointCloudBuffer().limit(); i++)
            {
               float x = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i);
               float y = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 1);
               float z = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 2);
               if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(y))
               {
                  Point3D point = ros2PointsToPublish.add();
                  point.set(x, y, z);
                  point.applyInverseTransform(sensorFrame.getTransformToRoot());
                  if (ros2ColorsToPublish != null)
                     ros2ColorsToPublish[i] = depthSensorSimulator.getColorRGBA8Buffer().getInt(Integer.BYTES * i);
               }
            }

            if (!ros2PointsToPublish.isEmpty())
            {
               pointCloudExecutor.execute(() ->
               {
                  long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
                  tempSensorFramePose.setToZero(sensorFrame);
                  tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  if (pointCloudMessageType.equals(LidarScanMessage.class))
                  {
                     LidarScanMessage message = PointCloudMessageTools.toLidarScanMessage(timestamp,
                                                                                          ros2PointsToPublish,
                                                                                          tempSensorFramePose);
                     ((ROS2PublisherBasics<LidarScanMessage>) publisher).publish(message);
                  }
                  else if (pointCloudMessageType.equals(StereoVisionPointCloudMessage.class))
                  {
                     int size = ros2PointsToPublish.size();
                     Point3D[] points = ros2PointsToPublish.toArray(new Point3D[size]);
                     int[] colors = Arrays.copyOf(ros2ColorsToPublish, size);
                     double minimumResolution = 0.005;
                     StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(timestamp,
                                                                                                            points,
                                                                                                            colors,
                                                                                                            size,
                                                                                                            minimumResolution,
                                                                                                            null);
                     message.getSensorPosition().set(tempSensorFramePose.getPosition());
                     message.getSensorOrientation().set(tempSensorFramePose.getOrientation());
                     message.setIsDataLocalToSensor(false);
                     //      LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
                     ((ROS2PublisherBasics<StereoVisionPointCloudMessage>) publisher).publish(message);
                  }
               });
            }
         }
         else if (pointCloudMessageType.equals(FusedSensorHeadPointCloudMessage.class))
         {
            Instant now = Instant.now();

            sensorFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
            float verticalFieldOfView = getLowLevelSimulator().getCamera().fieldOfView;
            float horizontalFieldOfView = verticalFieldOfView * imageWidth / imageHeight;
            float discreteResolution = 0.003f;
            int numberOfSegments = segmentationDivisor.get();
            int pointsPerSegment = depthSensorSimulator.getNumberOfPoints() / numberOfSegments;
            parametersBuffer.getBytedecoFloatBufferPointer().put(0, RDXPointCloudRenderer.FLOATS_PER_VERTEX);
            parametersBuffer.getBytedecoFloatBufferPointer().put(1, DiscretizedColoredPointCloud.DISCRETE_INTS_PER_POINT);
            parametersBuffer.getBytedecoFloatBufferPointer().put(2, discreteResolution);
            parametersBuffer.getBytedecoFloatBufferPointer().put(3, (float) segmentIndex);
            parametersBuffer.getBytedecoFloatBufferPointer().put(4, (float) pointsPerSegment);

            worldPointCloudBuffer.writeOpenCLBufferObject(openCLManager);
            parametersBuffer.writeOpenCLBufferObject(openCLManager);

            discretizedIntBuffer.getBackingDirectByteBuffer().rewind();

            openCLManager.setKernelArgument(discretizePointsKernel, 0, worldPointCloudBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(discretizePointsKernel, 1, discretizedIntBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(discretizePointsKernel, 2, parametersBuffer.getOpenCLBufferObject());
            openCLManager.execute1D(discretizePointsKernel, pointsPerSegment);
            discretizedIntBuffer.readOpenCLBufferObject(openCLManager);

            compressedPointCloudBuffer.rewind();
            compressedPointCloudBuffer.limit(compressedPointCloudBuffer.capacity());
            discretizedIntBuffer.getBackingDirectByteBuffer().rewind();
            // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
            lz4Compressor.compress(discretizedIntBuffer.getBackingDirectByteBuffer(), compressedPointCloudBuffer);
            compressedPointCloudBuffer.flip();
            outputFusedROS2Message.getScan().clear();
            for (int j = 0; j < compressedPointCloudBuffer.limit(); j++)
            {
               outputFusedROS2Message.getScan().add(compressedPointCloudBuffer.get());
            }
            outputFusedROS2Message.setAquisitionSecondsSinceEpoch(now.getEpochSecond());
            outputFusedROS2Message.setAquisitionAdditionalNanos(now.getNano());
            outputFusedROS2Message.setPointsPerSegment(pointsPerSegment);
            outputFusedROS2Message.setSegmentIndex(segmentIndex);
            outputFusedROS2Message.setNumberOfSegments(numberOfSegments);
            ((ROS2PublisherBasics<FusedSensorHeadPointCloudMessage>) publisher).publish(outputFusedROS2Message);

            ++segmentIndex;
            if (segmentIndex == numberOfSegments)
               segmentIndex = 0;
         }
      }
   }

   public void publishROS2DepthImageMessage()
   {
      if (!depthExecutor.isExecuting() && publishDepthImageMessageROS2.get())
      {
         depthExecutor.execute(() -> {

            OpenCVTools.convertFloatToShort(depthSensorSimulator.getMetersDepthOpenCVMat(),
                                            depthImageMat, 1000.0, 0.0);

            depthImageMessage.setImageHeight(depthSensorSimulator.getImageHeight());
            depthImageMessage.setImageWidth(depthSensorSimulator.getImageWidth());
            depthImageMessage.setFocalLengthXPixels(depthSensorSimulator.getFocalLengthPixels().get());
            depthImageMessage.setFocalLengthYPixels(depthSensorSimulator.getFocalLengthPixels().get());
            depthImageMessage.setPrincipalPointXPixels(depthSensorSimulator.getPrincipalOffsetXPixels().get());
            depthImageMessage.setPrincipalPointYPixels(depthSensorSimulator.getPrincipalOffsetYPixels().get());

            Instant now = Instant.now();
            sensorPose.setToZero(sensorFrame);
            sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
            OpenCVTools.compressImagePNG(depthImageMat, compressedDepthPointer);
            PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer, ros2DepthTopic, depthImageMessage, ros2Helper, sensorPose, now, depthSequenceNumber++,
                                                                   depthSensorSimulator.getImageHeight(), depthSensorSimulator.getImageWidth(), 0.001f);

         });
      }
   }

   public void publishROS2ColorImageMessage()
   {
      if (!colorExecutor.isExecuting() && publishColorImageMessageROS2.get())
      {
         colorExecutor.execute(() -> {

            colorImageMessage.setImageHeight(depthSensorSimulator.getImageHeight());
            colorImageMessage.setImageWidth(depthSensorSimulator.getImageWidth());
            colorImageMessage.setFocalLengthXPixels(depthSensorSimulator.getFocalLengthPixels().get());
            colorImageMessage.setFocalLengthYPixels(depthSensorSimulator.getFocalLengthPixels().get());
            colorImageMessage.setPrincipalPointXPixels(depthSensorSimulator.getPrincipalOffsetXPixels().get());
            colorImageMessage.setPrincipalPointYPixels(depthSensorSimulator.getPrincipalOffsetYPixels().get());

            opencv_imgproc.cvtColor(rgba8Mat, rgb8Mat, opencv_imgproc.COLOR_RGBA2RGB);

            Instant now = Instant.now();
            OpenCVTools.compressRGBImageJPG(rgb8Mat, yuv420Image, compressedColorPointer);
            PerceptionMessageTools.publishJPGCompressedColorImage(compressedColorPointer, ros2ColorTopic, colorImageMessage, ros2Helper, sensorPose, now, colorSequenceNumber++,
                                                                  depthSensorSimulator.getImageHeight(), depthSensorSimulator.getImageWidth(), 0.001f);
         });
      }

   }

   private void publishPointCloudROS1()
   {
      if (!pointCloudExecutor.isExecuting())
      {
         ros1PointsToPublish.clear();
         for (int i = 0; i < depthSensorSimulator.getNumberOfPoints()
                         && (FLOATS_PER_POINT * i + 2) < depthSensorSimulator.getPointCloudBuffer().limit(); i++)
         {
            float x = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i);
            float y = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 1);
            float z = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 2);
            ros1PointsToPublish.add().set(x, y, z);
         }

         if (!ros1PointsToPublish.isEmpty())
         {
            pointCloudExecutor.execute(() ->
            {
//               long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
//               tempSensorFramePose.setToZero(sensorFrame);
//               tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());

               int size = ros1PointsToPublish.size();
               Point3D[] points = ros1PointsToPublish.toArray(new Point3D[size]);

               ros1PointCloudPublisher.publish(points, new float[0], "os_sensor");
            });
         }
      }
   }

   public List<Point3D> getPointCloud()
   {
      pointCloud.clear();
      for (int i = 0; i < depthSensorSimulator.getNumberOfPoints()
                      && (FLOATS_PER_POINT * i + 2) < depthSensorSimulator.getPointCloudBuffer().limit(); i++)
      {
         float x = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i);
         float y = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 1);
         float z = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 2);
         pointCloud.add(new Point3D(x, y, z));
      }

      return pointCloud;
   }

   public void dispose()
   {
      if (realtimeROS2Node != null)
         realtimeROS2Node.destroy();
      depthExecutor.destroy();
      colorExecutor.destroy();
      pointCloudExecutor.destroy();
      colorROS2Executor.destroy();
      depthSensorSimulator.dispose();
      if (openCLManager != null)
         openCLManager.destroy();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevelsToRender)
   {
      if (sceneLevelsToRender.contains(RDXSceneLevel.MODEL))
      {
         if (renderPointCloudDirectly.get())
            pointCloudRenderer.getRenderables(renderables, pool);
      }
      if (sceneLevelsToRender.contains(RDXSceneLevel.VIRTUAL))
      {
         if (debugCoordinateFrame.get())
         {
            if (coordinateFrame == null)
               coordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.2);
            coordinateFrame.getRenderables(renderables, pool);
         }
         depthSensorSimulator.getVirtualRenderables(renderables, pool);
      }
   }

   public void setSensorEnabled(boolean sensorEnabled)
   {
      this.sensorEnabled.set(sensorEnabled);
   }

   public void setRenderPointCloudDirectly(boolean renderPointCloudDirectly)
   {
      this.renderPointCloudDirectly.set(renderPointCloudDirectly);
   }

   public void setPointSize(double size)
   {
      pointSize.set((float) size);
   }

   public void setRenderDepthVideoDirectly(boolean renderDepthVideoDirectly)
   {
      getLowLevelSimulator().getDepthPanel().getIsShowing().set(renderDepthVideoDirectly);
   }

   public void setRenderColorVideoDirectly(boolean renderColorVideoDirectly)
   {
      getLowLevelSimulator().getColorPanel().getIsShowing().set(renderColorVideoDirectly);
   }

   public void setPublishDepthImageROS1(boolean publish)
   {
      publishDepthImageROS1.set(publish);
   }

   public void setPublishColorImageROS1(boolean publish)
   {
      publishColorImageROS1.set(publish);
   }

   public void setPublishPointCloudROS1(boolean publish)
   {
      publishPointCloudROS1.set(publish);
   }

   public void setPublishColorImageROS2(boolean publish)
   {
      publishColorImageROS2.set(publish);
   }

   public void setPublishPointCloudROS2(boolean publish)
   {
      publishPointCloudROS2.set(publish);
   }

   public void setPublishDepthImageMessageROS2(boolean publish)
   {
      publishDepthImageMessageROS2.set(publish);
   }

   public void setPublishColorImageMessageROS2(boolean publish)
   {
      publishColorImageMessageROS2.set(publish);
   }

   public void setUseSensorColor(boolean useSensorColor)
   {
      this.useSensorColor.set(useSensorColor);
   }

   public void setDebugCoordinateFrame(boolean debugCoordinateFrame)
   {
      this.debugCoordinateFrame.set(debugCoordinateFrame);
   }

   public void setSensorFrameToWorldTransform(RigidBodyTransform sensorFrameToWorldTransform)
   {
      this.sensorFrameToWorldTransform = sensorFrameToWorldTransform;
   }

   public RDXLowLevelDepthSensorSimulator getLowLevelSimulator()
   {
      return depthSensorSimulator;
   }

   public CameraPinholeBrown getDepthCameraIntrinsics()
   {
      return depthCameraIntrinsics;
   }

   public CameraIntrinsics getCopyOfCameraParameters()
   {
      return new CameraIntrinsics(imageHeight,
                                  imageWidth,
                                  getLowLevelSimulator().getFocalLengthPixels().get(),
                                  getLowLevelSimulator().getFocalLengthPixels().get(),
                                  getLowLevelSimulator().getPrincipalOffsetXPixels().get(),
                                  getLowLevelSimulator().getPrincipalOffsetYPixels().get());
   }

   public Color getPointColorFromPicker()
   {
      return pointColorFromPicker;
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }
}
