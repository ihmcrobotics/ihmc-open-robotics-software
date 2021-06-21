package us.ihmc.gdx.simulation.sensors;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableInt;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.simulation.GDXLowLevelImageSensorSimulator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
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

import java.nio.IntBuffer;
import java.util.function.LongSupplier;

public class GDXHighLevelImageSensorSimulator extends ImGuiPanel implements RenderableProvider
{
   private static final MutableInt INDEX = new MutableInt();
   private final ReferenceFrame sensorFrame;
   private final Matrix4 gdxTransform = new Matrix4();
   private final GDXLowLevelImageSensorSimulator imageSensorSimulator;
   private final LongSupplier timestampSupplier;
   private final int imageWidth;
   private final int imageHeight;

   private final RosNodeInterface ros1Node;
   private RosImagePublisher ros1ColorPublisher;
   private RosCameraInfoPublisher ros1ColorCameraInfoPublisher;
   private ChannelBuffer ros1ColorChannelBuffer;

   private final ROS2NodeInterface ros2Node;
   private IHMCROS2Publisher<?> publisher;

   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService colorExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final double publishRateHz;
   private boolean debugCoordinateFrame;
   private ModelInstance coordinateFrame;
   private RigidBodyTransform sensorFrameToWorldTransform;

   private final ImBoolean sensorEnabled = new ImBoolean(false);
   private final ImBoolean renderColorVideoDirectly = new ImBoolean(false);
   private final ImBoolean publishColorImageROS1 = new ImBoolean(false);
   private final ImBoolean publishColorImageROS2 = new ImBoolean(false);

   public GDXHighLevelImageSensorSimulator(String sensorName,
                                           RosNodeInterface ros1Node,
                                           String ros1ColorImageTopic,
                                           String ros1ColorCameraInfoTopic,
                                           ROS2NodeInterface ros2Node,
                                           ROS2Topic<?> ros2Topic,
                                           ReferenceFrame sensorFrame,
                                           LongSupplier timestampSupplier,
                                           double verticalFOV,
                                           int imageWidth,
                                           int imageHeight,
                                           double minRange,
                                           double maxRange,
                                           double publishRateHz)
   {
      super(ImGuiTools.uniqueLabel(INDEX.getAndIncrement(), sensorName + " Simulator"));
      setRenderMethod(this::renderImGuiWidgets);
      this.ros1Node = ros1Node;
      this.ros2Node = ros2Node;
      this.sensorFrame = sensorFrame;
      this.timestampSupplier = timestampSupplier;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.publishRateHz = publishRateHz;

      imageSensorSimulator = new GDXLowLevelImageSensorSimulator(sensorName, verticalFOV, imageWidth, imageHeight, minRange, maxRange);

      if (ros1Node != null)
      {
         LogTools.info("Publishing ROS 1 color: {} {}", ros1ColorImageTopic, ros1ColorCameraInfoTopic);
         ros1ColorPublisher = new RosImagePublisher();
         ros1ColorCameraInfoPublisher = new RosCameraInfoPublisher();
         ros1Node.attachPublisher(ros1ColorCameraInfoTopic, ros1ColorCameraInfoPublisher);
         ros1Node.attachPublisher(ros1ColorImageTopic, ros1ColorPublisher);
         ros1ColorChannelBuffer = ros1ColorPublisher.getChannelBufferFactory().getBuffer(4 * imageWidth * imageHeight);
      }
      if (ros2Node != null)
      {
         LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
         publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());
      }

      throttleTimer.reset();
   }

   public void create()
   {
      imageSensorSimulator.create();
      addChild(imageSensorSimulator.getColorPanel());
      if (debugCoordinateFrame)
         coordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.2);
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      if (sensorEnabled.get())
      {
         if(sensorFrame != null)
            GDXTools.toGDX(sensorFrame.getTransformToWorldFrame(), gdxTransform);
         else
            GDXTools.toGDX(sensorFrameToWorldTransform, gdxTransform);

         imageSensorSimulator.setCameraWorldTransform(gdxTransform);
         imageSensorSimulator.render(sceneManager);

         if (coordinateFrame != null)
            coordinateFrame.transform.set(gdxTransform);

         if (throttleTimer.isExpired(UnitConversions.hertzToSeconds(publishRateHz)))
         {
            if (ros1Node != null)
            {
               if (publishColorImageROS1.get())
                  publishColorImageROS1();
            }
            if (ros2Node != null)
            {

            }
            throttleTimer.reset();
         }
      }
   }

   private void publishColorImageROS1()
   {
      if (ros1ColorPublisher.isConnected() && ros1ColorCameraInfoPublisher.isConnected() && !colorExecutor.isExecuting())
      {
         IntBuffer colorRGB8Buffer = imageSensorSimulator.getColorRGB8Buffer();
         colorRGB8Buffer.rewind();

         int bytesPerPixel = 4;
         ros1ColorChannelBuffer.clear();
         int size = 4 * imageWidth * imageHeight;
         for (int y = 0; y < imageHeight; y++)
         {
            for (int x = 0; x < imageWidth; x++)
            {
               try
               {
                  ros1ColorChannelBuffer.writeInt(colorRGB8Buffer.get());
               }
               catch (IndexOutOfBoundsException e)
               {
                  System.err.println(e.getMessage());
               }
            }
         }

         ros1ColorChannelBuffer.readerIndex(0);
         ros1ColorChannelBuffer.writerIndex(size);

         colorExecutor.execute(() ->
         {
            Image message = ros1ColorPublisher.createMessage(imageWidth, imageHeight, bytesPerPixel, "rgb8", ros1ColorChannelBuffer);
            if (timestampSupplier != null)
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));
            ros1ColorPublisher.publish(message);
         });
      }
   }


   public void renderImGuiWidgets()
   {
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Sensor Enabled"), sensorEnabled);
      ImGui.text("Render:");
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color video"), renderColorVideoDirectly);
      ImGui.text("Publish:");
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color image (ROS 1)"), publishColorImageROS1);
      ImGui.sameLine();
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Color image (ROS 2)"), publishColorImageROS2);
   }

   private void publishImageROS2()
   {
   }

   public void dispose()
   {
      colorExecutor.destroy();
      imageSensorSimulator.dispose();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (debugCoordinateFrame)
         coordinateFrame.getRenderables(renderables, pool);
   }

   public void setSensorEnabled(boolean sensorEnabled)
   {
      this.sensorEnabled.set(sensorEnabled);
   }


   public void setRenderColorVideoDirectly(boolean renderColorVideoDirectly)
   {
      this.imageSensorSimulator.getColorPanel().getEnabled().set(renderColorVideoDirectly);
   }

   public void setPublishColorImageROS2(boolean publish)
   {
      publishColorImageROS2.set(publish);
   }

   public void setPublishColorImageROS1(boolean publish)
   {
      publishColorImageROS1.set(publish);
   }

   public void setDebugCoordinateFrame(boolean debugCoordinateFrame)
   {
      this.debugCoordinateFrame = debugCoordinateFrame;
   }

   public void setSensorFrameToWorldTransform(RigidBodyTransform sensorFrameToWorldTransform)
   {
      this.sensorFrameToWorldTransform = sensorFrameToWorldTransform;
   }

   public GDXLowLevelImageSensorSimulator getLowLevelSimulator()
   {
      return imageSensorSimulator;
   }
}
