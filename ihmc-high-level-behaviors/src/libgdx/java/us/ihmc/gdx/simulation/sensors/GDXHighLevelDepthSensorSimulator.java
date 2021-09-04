package us.ihmc.gdx.simulation.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.VideoPacket;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.apache.commons.lang3.mutable.MutableInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.JPEGCompressor;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.simulation.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.function.LongSupplier;

public class GDXHighLevelDepthSensorSimulator extends ImGuiPanel implements RenderableProvider
{
   private static final MutableInt INDEX = new MutableInt();
   private final ReferenceFrame sensorFrame;
   private final Matrix4 gdxTransform = new Matrix4();
   private final GDXLowLevelDepthSensorSimulator depthSensorSimulator;
   private final LongSupplier timestampSupplier;
   private final CameraPinholeBrown depthCameraIntrinsics;
   private final int imageWidth;
   private final int imageHeight;
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();

   private final RosNodeInterface ros1Node;
   private final String ros1DepthImageTopic;
   private final String ros1ColorImageTopic;
   private final ROS2Topic<?> ros2PointCloudTopic;
   private RosImagePublisher ros1DepthPublisher;
   private RosCameraInfoPublisher ros1DepthCameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;
   private RosImagePublisher ros1ColorPublisher;
   private RosCameraInfoPublisher ros1ColorCameraInfoPublisher;
   private ChannelBuffer ros1ColorChannelBuffer;

   private final ROS2NodeInterface ros2Node;
   private boolean ros2IsLidarScan;
   private IHMCROS2Publisher<?> publisher;
   private IHMCROS2Publisher<VideoPacket> ros2VideoPublisher;
   private ByteBuffer flippedColorByteBuffer;
   private final JPEGCompressor jpegCompressor = new JPEGCompressor();
   private RecyclingArrayList<Point3D> ros2PointsToPublish;
   private int[] ros2ColorsToPublish;
   private final FramePose3D tempSensorFramePose = new FramePose3D();
   private final FramePose3D tempSensorFramePose2 = new FramePose3D();

   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService depthExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService pointCloudExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorROS2Executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final double publishRateHz;
   private final ImBoolean debugCoordinateFrame = new ImBoolean(false);
   private ModelInstance coordinateFrame;
   private RigidBodyTransform sensorFrameToWorldTransform;

   private boolean tuning = false;
   private final ImBoolean sensorEnabled = new ImBoolean(false);
   private final ImBoolean renderPointCloudDirectly = new ImBoolean(false);
   private final ImBoolean publishDepthImageROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS2 = new ImBoolean(false);
   private final ImBoolean publishPointCloudROS2 = new ImBoolean(false);
   private final ImFloat fx;
   private final ImFloat fy;
   private final ImFloat skew;
   private final ImFloat cx;
   private final ImFloat cy;

   private Mat rgba8Mat;
   private Mat rgb8Mat;
   private ByteBuffer rgb8Buffer;

   public GDXHighLevelDepthSensorSimulator(String sensorName,
                                           RosNodeInterface ros1Node,
                                           String ros1DepthImageTopic,
                                           String ros1DepthCameraInfoTopic,
                                           CameraPinholeBrown depthCameraIntrinsics,
                                           String ros1ColorImageTopic,
                                           String ros1ColorCameraInfoTopic,
                                           ROS2NodeInterface ros2Node,
                                           ROS2Topic<?> ros2PointCloudTopic,
                                           ROS2Topic<VideoPacket> ros2VideoTopic,
                                           ReferenceFrame sensorFrame,
                                           LongSupplier timestampSupplier,
                                           double verticalFOV,
                                           int imageWidth,
                                           int imageHeight,
                                           double minRange,
                                           double maxRange,
                                           double publishRateHz,
                                           boolean renderPointCloudDirectly)
   {
      super(ImGuiTools.uniqueLabel(INDEX.getAndIncrement(), sensorName + " Simulator"));
      setRenderMethod(this::renderImGuiWidgets);
      this.ros1Node = ros1Node;
      this.ros1DepthImageTopic = ros1DepthImageTopic;
      this.depthCameraIntrinsics = depthCameraIntrinsics;
      this.ros1ColorImageTopic = ros1ColorImageTopic;
      this.ros2Node = ros2Node;
      this.ros2PointCloudTopic = ros2PointCloudTopic;
      this.sensorFrame = sensorFrame;
      this.timestampSupplier = timestampSupplier;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.publishRateHz = publishRateHz;
      this.renderPointCloudDirectly.set(renderPointCloudDirectly);

      fx = new ImFloat((float) depthCameraIntrinsics.getFx());
      fy = new ImFloat((float) depthCameraIntrinsics.getFy());
      skew = new ImFloat((float) depthCameraIntrinsics.getSkew());
      cx = new ImFloat((float) depthCameraIntrinsics.getCx());
      cy = new ImFloat((float) depthCameraIntrinsics.getCy());

      depthSensorSimulator = new GDXLowLevelDepthSensorSimulator(sensorName, verticalFOV, imageWidth, imageHeight, minRange, maxRange);

      if (ros1Node != null)
      {
         LogTools.info("Publishing ROS 1 depth: {} {}", ros1DepthImageTopic, ros1DepthCameraInfoTopic);
         ros1DepthPublisher = new RosImagePublisher();
         ros1DepthCameraInfoPublisher = new RosCameraInfoPublisher();
         ros1Node.attachPublisher(ros1DepthCameraInfoTopic, ros1DepthCameraInfoPublisher);
         ros1Node.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
         ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * imageWidth * imageHeight);
         LogTools.info("Publishing ROS 1 color: {} {}", ros1ColorImageTopic, ros1ColorCameraInfoTopic);
         ros1ColorPublisher = new RosImagePublisher();
         ros1ColorCameraInfoPublisher = new RosCameraInfoPublisher();
         ros1Node.attachPublisher(ros1ColorCameraInfoTopic, ros1ColorCameraInfoPublisher);
         ros1Node.attachPublisher(ros1ColorImageTopic, ros1ColorPublisher);
         ros1ColorChannelBuffer = ros1ColorPublisher.getChannelBufferFactory().getBuffer(3 * imageWidth * imageHeight);
      }
      if (ros2Node != null && ros2PointCloudTopic != null)
      {
         ros2PointsToPublish = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D::new);
         ros2IsLidarScan = ros2PointCloudTopic.getType().equals(LidarScanMessage.class);
         if (!ros2IsLidarScan)
            ros2ColorsToPublish = new int[imageWidth * imageHeight];
         LogTools.info("Publishing ROS 2: {}", ros2PointCloudTopic.getName());
         publisher = ROS2Tools.createPublisher(ros2Node, ros2PointCloudTopic, ROS2QosProfile.DEFAULT());
      }
      if (ros2Node != null && ros2VideoTopic != null)
      {
         ros2VideoPublisher = ROS2Tools.createPublisher(ros2Node, ros2VideoTopic);
         flippedColorByteBuffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 4);
      }

      throttleTimer.reset();
   }

   public void create()
   {
      depthSensorSimulator.create();
      addChild(depthSensorSimulator.getDepthPanel());
      addChild(depthSensorSimulator.getColorPanel());
      pointCloudRenderer.create(imageWidth * imageHeight);
      if (debugCoordinateFrame.get())
         coordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.2);

      rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new BytePointer(depthSensorSimulator.getColorRGBA8Buffer()));
      rgb8Buffer = BufferUtils.newByteBuffer(imageWidth * imageHeight * 3);
      rgb8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3, new BytePointer(rgb8Buffer));
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      if (sensorEnabled.get())
      {
         if(sensorFrame != null)
            GDXTools.toGDX(sensorFrame.getTransformToWorldFrame(), gdxTransform);
         else
            GDXTools.toGDX(sensorFrameToWorldTransform, gdxTransform);

         depthSensorSimulator.setCameraWorldTransform(gdxTransform);
         depthSensorSimulator.render(sceneManager);

         if (coordinateFrame != null)
            coordinateFrame.transform.set(gdxTransform);

         if (renderPointCloudDirectly.get())
         {
            pointCloudRenderer.setPointsToRender(depthSensorSimulator.getPoints());
            pointCloudRenderer.updateMesh();
         }
         if (throttleTimer.isExpired(UnitConversions.hertzToSeconds(publishRateHz)))
         {
            if (ros1Node != null)
            {
               if (publishDepthImageROS1.get())
                  publishDepthImageROS1();
               if (publishColorImageROS1.get())
                  publishColorImageROS1();
            }
            if (ros2Node != null)
            {
               if (publishPointCloudROS2.get())
                  publishPointCloudROS2();
               if (publishColorImageROS2.get())
                  publishColorImageROS2();
            }
            throttleTimer.reset();
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
         ByteBuffer colorRGBA8Buffer = depthSensorSimulator.getColorRGBA8Buffer();
         colorRGBA8Buffer.rewind();
         int capacity = colorRGBA8Buffer.capacity();
         for (int y = 0; y < imageHeight; y++)
         {
            for (int x = 0; x < imageWidth; x++)
//            for (int i = 0; i < capacity; i += 4)
            {
               int iSrc = (y * imageWidth + x) * 4;
               int iDest = (y * imageWidth + (imageWidth - x - 1)) * 4;

               // flipping RGBA to BGRA here
               flippedColorByteBuffer.put(capacity - iDest - 4 + 0, colorRGBA8Buffer.get(iSrc + 2));
               flippedColorByteBuffer.put(capacity - iDest - 4 + 1, colorRGBA8Buffer.get(iSrc + 1));
               flippedColorByteBuffer.put(capacity - iDest - 4 + 2, colorRGBA8Buffer.get(iSrc + 0));
               flippedColorByteBuffer.put(capacity - iDest - 4 + 3, colorRGBA8Buffer.get(iSrc + 3));
            }
         }
         flippedColorByteBuffer.rewind();

         byte[] data = jpegCompressor.convertBGRA8ToJPEGData(imageWidth, imageHeight, flippedColorByteBuffer);

         colorROS2Executor.execute(() ->
         {
            VideoPacket videoPacket = new VideoPacket();
            videoPacket.setVideoSource(VideoPacket.VIDEO_SOURCE_MULTISENSE_LEFT_EYE);
            long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
            videoPacket.setTimestamp(timestamp);
            tempSensorFramePose2.setToZero(sensorFrame);
            tempSensorFramePose2.changeFrame(ReferenceFrame.getWorldFrame());
            videoPacket.getPosition().set(tempSensorFramePose2.getPosition());
            videoPacket.getOrientation().set(tempSensorFramePose2.getOrientation());
            videoPacket.getIntrinsicParameters().set(HumanoidMessageTools.toIntrinsicParametersMessage(depthCameraIntrinsics));
            videoPacket.getData().add(data);
            ros2VideoPublisher.publish(videoPacket);
         });
      }
   }

   private void publishDepthImageROS1()
   {
      if (ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected() && !depthExecutor.isExecuting())
      {
         PerspectiveCamera camera = depthSensorSimulator.getCamera();
         FloatBuffer depthFloatBuffer = depthSensorSimulator.getEyeDepthMetersBuffer();
         depthFloatBuffer.rewind();
         ros1DepthChannelBuffer.clear();
         int size = 2 * imageWidth * imageHeight;
         for (int y = 0; y < imageHeight; y++)
         {
            for (int x = 0; x < imageWidth; x++)
            {
               float eyeDepthMeters = depthFloatBuffer.get();

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
            {
               depthCameraIntrinsics.setFx(fx.get());
               depthCameraIntrinsics.setFy(fy.get());
               depthCameraIntrinsics.setSkew(skew.get());
               depthCameraIntrinsics.setCx(cx.get());
               depthCameraIntrinsics.setCx(cx.get());
            }
            ros1DepthCameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time());
            Image message = ros1DepthPublisher.createMessage(imageWidth, imageHeight, 2, "16UC1", ros1DepthChannelBuffer); // maybe need to copy here if there are errors

            if(timestampSupplier != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));

            ros1DepthPublisher.publish(message);
         });
      }
   }

   public void renderImGuiWidgets()
   {
      tuning = true;
      ImGui.text("Resolution: " + imageWidth + " x " + imageWidth);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Sensor Enabled"), sensorEnabled);
      ImGui.sameLine();
      ImGui.checkbox("Show frame graphic", debugCoordinateFrame);
      ImGui.text("Render:");
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Point cloud"), renderPointCloudDirectly);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Depth video"), getLowLevelSimulator().getDepthPanel().getIsShowing());
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color video"), getLowLevelSimulator().getColorPanel().getIsShowing());
      ImGui.text("Publish:");
      if (ros1DepthImageTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 1 Depth image (" + ros1DepthImageTopic + ")"), publishDepthImageROS1);
      if (ros1ColorImageTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 1 Color image (" + ros1ColorImageTopic + ")"), publishColorImageROS1);
      if (ros2PointCloudTopic != null)
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "ROS 2 Point cloud (" + ros2PointCloudTopic + ")"), publishPointCloudROS2);
      if (flippedColorByteBuffer != null)
      {
         ImGui.sameLine();
         ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color image (ROS 2)"), publishColorImageROS2);
      }
      if (ImGui.collapsingHeader("Tuning sliders"))
      {
         getLowLevelSimulator().renderTuningSliders();
         ImGui.sliderFloat("Fx", fx.getData(), -1000.0f, 1000.0f);
         ImGui.sliderFloat("Fy", fy.getData(), -1000.0f, 1000.0f);
         ImGui.sliderFloat("Skew", skew.getData(), -1000.0f, 1000.0f);
         ImGui.sliderFloat("Cx", cx.getData(), -1000.0f, 1000.0f);
         ImGui.sliderFloat("Cy", cy.getData(), -1000.0f, 1000.0f);
      }
   }

   private void publishPointCloudROS2()
   {
      if (!pointCloudExecutor.isExecuting())
      {
         ros2PointsToPublish.clear();
         for (int i = 0; i < depthSensorSimulator.getPoints().size(); i++)
         {
            ros2PointsToPublish.add().set(depthSensorSimulator.getPoints().get(i));
            if (ros2ColorsToPublish != null)
               ros2ColorsToPublish[i] = depthSensorSimulator.getColors().get(i);
         }

         if (!ros2PointsToPublish.isEmpty())
         {
            pointCloudExecutor.execute(() ->
            {
               long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
               tempSensorFramePose.setToZero(sensorFrame);
               tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               if (ros2IsLidarScan)
               {
                  LidarScanMessage message = PointCloudMessageTools.toLidarScanMessage(timestamp,
                                                                                       ros2PointsToPublish,
                                                                                       tempSensorFramePose);
                  ((IHMCROS2Publisher<LidarScanMessage>) publisher).publish(message);
               }
               else
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
                  //      LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
                  ((IHMCROS2Publisher<StereoVisionPointCloudMessage>) publisher).publish(message);
               }
            });
         }
      }
   }

   public void dispose()
   {
      depthExecutor.destroy();
      colorExecutor.destroy();
      pointCloudExecutor.destroy();
      colorROS2Executor.destroy();
      depthSensorSimulator.dispose();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderPointCloudDirectly.get())
         pointCloudRenderer.getRenderables(renderables, pool);
      if (debugCoordinateFrame.get())
      {
         if (coordinateFrame == null)
            coordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.2);
         coordinateFrame.getRenderables(renderables, pool);
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

   public void setPublishColorImageROS2(boolean publish)
   {
      publishColorImageROS2.set(publish);
   }

   public void setPublishPointCloudROS2(boolean publish)
   {
      publishPointCloudROS2.set(publish);
   }

   public void setDebugCoordinateFrame(boolean debugCoordinateFrame)
   {
      this.debugCoordinateFrame.set(debugCoordinateFrame);
   }

   public void setSensorFrameToWorldTransform(RigidBodyTransform sensorFrameToWorldTransform)
   {
      this.sensorFrameToWorldTransform = sensorFrameToWorldTransform;
   }

   public RecyclingArrayList<Point3D32> getPoints()
   {
      return depthSensorSimulator.getPoints();
   }

   public GDXLowLevelDepthSensorSimulator getLowLevelSimulator()
   {
      return depthSensorSimulator;
   }
}
