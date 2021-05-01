package us.ihmc.gdx.simulation.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.apache.commons.lang3.mutable.MutableInt;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.simulation.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
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

import java.nio.FloatBuffer;
import java.util.Arrays;

public class GDXHighLevelDepthSensorSimulator implements RenderableProvider
{
   private static final MutableInt INDEX = new MutableInt();
   private static final String WINDOW_NAME = "Depth Sensor";
   private final String windowName = ImGuiTools.uniqueLabel(INDEX.getAndIncrement(), WINDOW_NAME);
   private final ReferenceFrame sensorFrame;
   private final Matrix4 gdxTransform = new Matrix4();
   private final GDXLowLevelDepthSensorSimulator depthSensorSimulator;
   private final RemoteSyncedRobotModel syncedRobot;
   private final CameraPinholeBrown depthCameraIntrinsics;
   private final int imageWidth;
   private final int imageHeight;
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();

   private final RosNodeInterface ros1Node;
   private RosImagePublisher ros1DepthPublisher;
   private RosCameraInfoPublisher ros1CameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;

   private final ROS2NodeInterface ros2Node;
   private boolean ros2IsLidarScan;
   private IHMCROS2Publisher<?> publisher;
   private RecyclingArrayList<Point3D> ros2PointsToPublish;
   private int[] ros2ColorsToPublish;
   private final FramePose3D tempSensorFramePose = new FramePose3D();

   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final double publishRateHz;
   private boolean debugCoordinateFrame;
   private ModelInstance coordinateFrame;
   private RigidBodyTransform sensorFrameToWorldTransform;

   private boolean tuning = false;
   private final ImBoolean sensorEnabled = new ImBoolean(false);
   private final ImBoolean renderPointCloudDirectly = new ImBoolean(false);
   private final ImBoolean renderDepthVideoDirectly = new ImBoolean(false);
   private final ImBoolean renderColorVideoDirectly = new ImBoolean(false);
   private final ImBoolean publishDepthImageROS1 = new ImBoolean(true);
   private final ImBoolean publishColorImageROS1 = new ImBoolean(true);
   private final ImBoolean publishPointCloudROS2 = new ImBoolean(true);
   private final ImFloat fx;
   private final ImFloat fy;
   private final ImFloat skew;
   private final ImFloat cx;
   private final ImFloat cy;

   public GDXHighLevelDepthSensorSimulator(RosNodeInterface ros1Node,
                                           String ros1DepthImageTopic,
                                           String ros1CameraInfoTopic,
                                           CameraPinholeBrown depthCameraIntrinsics,
                                           ROS2NodeInterface ros2Node,
                                           ROS2Topic<?> ros2Topic,
                                           ReferenceFrame sensorFrame,
                                           RemoteSyncedRobotModel syncedRobot,
                                           double verticalFOV,
                                           int imageWidth,
                                           int imageHeight,
                                           double minRange,
                                           double maxRange,
                                           double publishRateHz,
                                           boolean renderPointCloudDirectly)
   {
      this.ros1Node = ros1Node;
      this.depthCameraIntrinsics = depthCameraIntrinsics;
      this.ros2Node = ros2Node;
      this.sensorFrame = sensorFrame;
      this.syncedRobot = syncedRobot;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.publishRateHz = publishRateHz;
      this.renderPointCloudDirectly.set(renderPointCloudDirectly);

      fx = new ImFloat((float) depthCameraIntrinsics.getFx());
      fy = new ImFloat((float) depthCameraIntrinsics.getFy());
      skew = new ImFloat((float) depthCameraIntrinsics.getSkew());
      cx = new ImFloat((float) depthCameraIntrinsics.getCx());
      cy = new ImFloat((float) depthCameraIntrinsics.getCy());

      depthSensorSimulator = new GDXLowLevelDepthSensorSimulator(verticalFOV, imageWidth, imageHeight, minRange, maxRange);

      if (ros1Node != null)
      {
         LogTools.info("Publishing ROS 1: {} {}", ros1DepthImageTopic, ros1CameraInfoTopic);
         ros1DepthPublisher = new RosImagePublisher();
         ros1CameraInfoPublisher = new RosCameraInfoPublisher();
         ros1Node.attachPublisher(ros1CameraInfoTopic, ros1CameraInfoPublisher);
         ros1Node.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
         ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * imageWidth * imageHeight);
      }
      if (ros2Node != null)
      {
         ros2PointsToPublish = new RecyclingArrayList<>(imageWidth * imageHeight, Point3D::new);
         ros2IsLidarScan = ros2Topic.getType().equals(LidarScanMessage.class);
         if (!ros2IsLidarScan)
            ros2ColorsToPublish = new int[imageWidth * imageHeight];
         LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
         publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());
      }

      throttleTimer.reset();
   }

   public void create()
   {
      depthSensorSimulator.create();
      pointCloudRenderer.create(imageWidth * imageHeight);
      if (debugCoordinateFrame)
         coordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.2);
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      if (sensorEnabled.get())
      {
         if (syncedRobot != null)
            syncedRobot.update();

         if(sensorFrame != null)
            GDXTools.toGDX(sensorFrame.getTransformToWorldFrame(), gdxTransform);
         else
            GDXTools.toGDX(sensorFrameToWorldTransform, gdxTransform);

         depthSensorSimulator.setCameraWorldTransform(gdxTransform);
         depthSensorSimulator.render(sceneManager);

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
            }
            if (ros2Node != null)
            {
               if (publishPointCloudROS2.get())
                  publishPointCloudROS2();
            }
            throttleTimer.reset();
         }

         tuning = false;
      }
   }

   private void publishDepthImageROS1()
   {
      if (ros1DepthPublisher.isConnected() && ros1CameraInfoPublisher.isConnected() && !executor.isExecuting())
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

         executor.execute(() ->
         {
            if (tuning)
            {
               depthCameraIntrinsics.setFx(fx.get());
               depthCameraIntrinsics.setFy(fy.get());
               depthCameraIntrinsics.setSkew(skew.get());
               depthCameraIntrinsics.setCx(cx.get());
               depthCameraIntrinsics.setCx(cx.get());
            }
            ros1CameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time());
            Image message = ros1DepthPublisher.createMessage(imageWidth, imageHeight, ros1DepthChannelBuffer); // maybe need to copy here if there are errors

            if(syncedRobot != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp())));

            ros1DepthPublisher.publish(message);
         });
      }
   }

   public void renderImGuiWindow()
   {
      tuning = true;
      ImGui.begin(windowName);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Sensor Enabled"), sensorEnabled);
      ImGui.text("Render:");
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Point cloud"), renderPointCloudDirectly);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Depth video"), renderDepthVideoDirectly);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color video"), renderColorVideoDirectly);
      ImGui.text("Publish:");
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Depth image (ROS 1)"), publishDepthImageROS1);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color image (ROS 1)"), publishColorImageROS1);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Point cloud (ROS 2)"), publishPointCloudROS2);
      getLowLevelSimulator().renderTuningSliders();
      ImGui.sliderFloat("Fx", fx.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat("Fy", fy.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat("Skew", skew.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat("Cx", cx.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat("Cy", cy.getData(), -1000.0f, 1000.0f);
      ImGui.end();

      if (renderDepthVideoDirectly.get())
         getLowLevelSimulator().renderImGuiDepthWindow();
      if (renderColorVideoDirectly.get())
         getLowLevelSimulator().renderImGuiColorWindow();
   }

   private void publishPointCloudROS2()
   {
      if (!executor.isExecuting())
      {
         ros2PointsToPublish.clear();
         for (int i = 0; i < depthSensorSimulator.getPoints().size(); i++)
         {
            ros2PointsToPublish.add().set(depthSensorSimulator.getPoints().get(i));
            if (ros2ColorsToPublish != null)
               ros2ColorsToPublish[i] = depthSensorSimulator.getColors().get(i);
         }

         executor.execute(() ->
         {
            long timestamp = syncedRobot == null ? System.nanoTime() : syncedRobot.getTimestamp();
            tempSensorFramePose.setToZero(sensorFrame);
            tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            if (ros2IsLidarScan)
            {
               LidarScanMessage message = PointCloudMessageTools.toLidarScanMessage(timestamp, ros2PointsToPublish, tempSensorFramePose);
               ((IHMCROS2Publisher<LidarScanMessage>) publisher).publish(message);
            }
            else
            {
               int size = ros2PointsToPublish.size();
               Point3D[] points = ros2PointsToPublish.toArray(new Point3D[size]);
               int[] colors = Arrays.copyOf(ros2ColorsToPublish, size);
               double minimumResolution = 0.005;
               StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(timestamp, points, colors, size, minimumResolution, null);
               message.getSensorPosition().set(tempSensorFramePose.getPosition());
               message.getSensorOrientation().set(tempSensorFramePose.getOrientation());
               //      LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
               ((IHMCROS2Publisher<StereoVisionPointCloudMessage>) publisher).publish(message);
            }
         });
      }
   }

   public void dispose()
   {
      executor.destroy();
      depthSensorSimulator.dispose();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderPointCloudDirectly.get())
         pointCloudRenderer.getRenderables(renderables, pool);
      if (debugCoordinateFrame)
         coordinateFrame.getRenderables(renderables, pool);
   }

   public void setSensorEnabled(boolean sensorEnabled)
   {
      this.sensorEnabled.set(sensorEnabled);
   }

   public void setRenderPointCloudDirectly(boolean renderPointCloudDirectly)
   {
      this.renderPointCloudDirectly.set(renderPointCloudDirectly);
   }

   public void setDebugCoordinateFrame(boolean debugCoordinateFrame)
   {
      this.debugCoordinateFrame = debugCoordinateFrame;
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

   public String getWindowName()
   {
      return windowName;
   }
}
