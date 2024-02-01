package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXColorGradientMode;
import us.ihmc.rdx.ui.graphics.RDXOusterFisheyeColoredPointCloudKernel;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.Timer;
import us.ihmc.tools.string.StringTools;

import java.util.Set;

/**
 * Supports Realsenses and Ouster and Fisheye colored point clouds.
 * This visualizer is designed to automatically resize and adapt to changing
 * parameters, such as focal length, principal points, etc.
 * It provides our full set of analytics as plots, such as message size,
 * delay, sequence discontinuities, etc. and coloring options.
 */
public class RDXROS2ColoredPointCloudVisualizer extends RDXVisualizer
{
   private final String titleBeforeAdditions;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(false);
   private final ImBoolean useSensorColor = new ImBoolean(true);
   private RDXColorGradientMode gradientMode = RDXColorGradientMode.WORLD_Z;
   private final ImBoolean useSinusoidalGradientPattern = new ImBoolean(true);
   private final ImFloat pointSizeScale = new ImFloat(1.0f);
   private final ImInt levelOfColorDetail = new ImInt(0);

   private final RDXROS2ColoredPointCloudVisualizerDepthChannel depthChannel;
   private final RDXROS2ColoredPointCloudVisualizerColorChannel colorChannel;
   private final Timer colorReceptionTimer = new Timer();

   private final PubSubImplementation pubSubImplementation;
   private RealtimeROS2Node realtimeROS2Node;
   private final Object imageMessagesSyncObject = new Object();

   private RDXPointCloudRenderer pointCloudRenderer;
   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private RDXPinholePinholeColoredPointCloudKernel pinholePinholeKernel;
   private RDXOusterFisheyeColoredPointCloudKernel ousterFisheyeKernel;
   private boolean usingColor;

   private final MutableReferenceFrame depthFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame colorFrame = new MutableReferenceFrame();
   private final RigidBodyTransform depthToColorTransform = new RigidBodyTransform();


   public RDXROS2ColoredPointCloudVisualizer(String title,
                                             PubSubImplementation pubSubImplementation,
                                             ROS2Topic<ImageMessage> depthTopic,
                                             ROS2Topic<ImageMessage> colorTopic)
   {
      super(title + " (ROS 2)");
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      depthChannel = new RDXROS2ColoredPointCloudVisualizerDepthChannel(depthTopic);
      colorChannel = new RDXROS2ColoredPointCloudVisualizerColorChannel(colorTopic);
   }

   private void subscribe()
   {
      subscribed.set(true);
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      depthChannel.subscribe(realtimeROS2Node, imageMessagesSyncObject);
      colorChannel.subscribe(realtimeROS2Node, imageMessagesSyncObject);
      realtimeROS2Node.spin();
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      pinholePinholeKernel = new RDXPinholePinholeColoredPointCloudKernel(openCLManager);
      ousterFisheyeKernel = new RDXOusterFisheyeColoredPointCloudKernel(openCLManager);
   }

   @Override
   public void update()
   {
      super.update();

      if (subscribed.get() && isActive())
      {
         synchronized (imageMessagesSyncObject)
         {
            depthChannel.update(openCLManager);
            colorChannel.update(openCLManager);
         }

         if (depthChannel.getDecompressedImageReady().poll())
         {
            // Stop coloring points if there's been no data in a few seconds
            if (colorChannel.getDecompressedImageReady().poll())
               colorReceptionTimer.reset();
            usingColor = colorReceptionTimer.isRunning(2.0);

            int totalNumberOfPoints = depthChannel.getTotalNumberOfPixels();
            if (depthChannel.getCameraModel() == CameraModel.OUSTER)
            {
               int sanitizedLevelOfColorDetail = usingColor && colorChannel.getCameraModel() == CameraModel.EQUIDISTANT_FISHEYE ? levelOfColorDetail.get() : 0;
               totalNumberOfPoints = ousterFisheyeKernel.calculateNumberOfPointsForLevelOfColorDetail(depthChannel.getImageWidth(),
                                                                                                      depthChannel.getImageHeight(),
                                                                                                      sanitizedLevelOfColorDetail);
            }

            if (pointCloudVertexBuffer == null
             || pointCloudVertexBuffer.getBackingDirectFloatBuffer().capacity() / RDXPointCloudRenderer.FLOATS_PER_VERTEX != totalNumberOfPoints)
            {
               LogTools.info("Allocating new buffers. {} total points", totalNumberOfPoints);

               if (pointCloudRenderer != null)
                  pointCloudRenderer.dispose();
               pointCloudRenderer = new RDXPointCloudRenderer();
               pointCloudRenderer.create(totalNumberOfPoints);

               pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX, pointCloudRenderer.getVertexBuffer());
               pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
            }

            pointCloudRenderer.updateMeshFastestBeforeKernel();
            pointCloudVertexBuffer.syncWithBackingBuffer(); // TODO: Is this necessary?

            synchronized (depthChannel.getDecompressionAccessSyncObject())
            {
               if (usingColor)
               {
                  synchronized (colorChannel.getDecompressionAccessSyncObject())
                  {
                     runKernels();
                  }
               }
               else
               {
                  runKernels();
               }
            }

            pointCloudRenderer.updateMeshFastestAfterKernel();
         }
      }
   }

   private void runKernels()
   {
      float pointSize = pointSizeScale.get() / depthChannel.getFx();
      if (depthChannel.getCameraModel() == CameraModel.PINHOLE) // Assuming color camera is also pinhole if using it
      {
         pinholePinholeKernel.computeVertexBuffer(colorChannel,
                                                  depthChannel,
                                                  usingColor && useSensorColor.get(),
                                                  gradientMode.ordinal(),
                                                  useSinusoidalGradientPattern.get(),
                                                  pointSize,
                                                  pointCloudVertexBuffer);
      }
      else if (depthChannel.getCameraModel() == CameraModel.OUSTER) // Assuming color is equidistant fisheye if using it
      {
         depthFrame.update(transformToWorld -> transformToWorld.set(depthChannel.getRotationMatrixToWorld(), depthChannel.getTranslationToWorld()));
         colorFrame.update(transformToWorld -> transformToWorld.set(colorChannel.getRotationMatrixToWorld(), colorChannel.getTranslationToWorld()));
         depthFrame.getReferenceFrame().getTransformToDesiredFrame(depthToColorTransform, colorFrame.getReferenceFrame());

         ousterFisheyeKernel.getOusterToWorldTransformToPack().set(depthChannel.getRotationMatrixToWorld(), depthChannel.getTranslationToWorld());
         ousterFisheyeKernel.getOusterToFisheyeTransformToPack().set(depthToColorTransform.getRotation(), depthToColorTransform.getTranslation());
         ousterFisheyeKernel.setInstrinsicParameters(depthChannel.getOusterBeamAltitudeAnglesBuffer(), depthChannel.getOusterBeamAzimuthAnglesBuffer());
         ousterFisheyeKernel.runKernel(0.0f,
                                       pointSize,
                                       usingColor && useSensorColor.get(),
                                       gradientMode.ordinal(),
                                       useSinusoidalGradientPattern.get(),
                                       depthChannel.getDepth16UC1Image(),
                                       colorChannel.getFx(),
                                       colorChannel.getFy(),
                                       colorChannel.getCx(),
                                       colorChannel.getCy(),
                                       usingColor ? colorChannel.getColor8UC4Image() : null,
                                       pointCloudVertexBuffer);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.getHidden(getTitle() + "Subscribed"), subscribed))
      {
         setSubscribed(subscribed.get());
      }
      ImGuiTools.previousWidgetTooltip("Subscribed");
      ImGui.sameLine();
      super.renderImGuiWidgets();
      ImGui.text(colorChannel.getTopic().getName());

      renderStatistics();

      ImGui.checkbox(labels.get("Use sensor color"), useSensorColor);
      ImGui.text("Gradient mode:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World Z"), gradientMode == RDXColorGradientMode.WORLD_Z))
         gradientMode = RDXColorGradientMode.WORLD_Z;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Sensor X"), gradientMode == RDXColorGradientMode.SENSOR_X))
         gradientMode = RDXColorGradientMode.SENSOR_X;
      ImGui.checkbox(labels.get("Sinusoidal gradient"), useSinusoidalGradientPattern);
      ImGui.sliderFloat(labels.get("Point scale"), pointSizeScale.getData(), 0.0f, 2.0f);
      if (depthChannel.getCameraModel() == CameraModel.OUSTER && colorChannel.getCameraModel() == CameraModel.EQUIDISTANT_FISHEYE)
      {
         ImGui.sliderInt(labels.get("Level of color detail"), levelOfColorDetail.getData(), 0, 3);
      }
   }

   public void renderStatistics()
   {
      if (colorChannel.getReceivedOne())
      {
         colorChannel.getMessageSizeReadout().renderImGuiWidgets();
      }
      ImGui.text(depthChannel.getTopic().getName());
      if (depthChannel.getReceivedOne())
      {
         depthChannel.getMessageSizeReadout().renderImGuiWidgets();
      }
      if (colorChannel.getReceivedOne())
         colorChannel.getFrequencyPlot().renderImGuiWidgets();
      if (depthChannel.getReceivedOne())
         depthChannel.getFrequencyPlot().renderImGuiWidgets();

      if (colorChannel.getReceivedOne())
         colorChannel.getDelayPlot().renderImGuiWidgets();
      if (depthChannel.getReceivedOne())
         depthChannel.getDelayPlot().renderImGuiWidgets();

      if (colorChannel.getReceivedOne())
         colorChannel.getSequenceDiscontinuityPlot().renderImGuiWidgets();
      if (depthChannel.getReceivedOne())
         depthChannel.getSequenceDiscontinuityPlot().renderImGuiWidgets();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && pointCloudRenderer != null && sceneLevelCheck(sceneLevels))
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      depthChannel.destroy();
      colorChannel.destroy();
      unsubscribe();
      super.destroy();
   }

   public void setSubscribed(boolean subscribed)
   {
      if (subscribed && realtimeROS2Node == null)
      {
         subscribe();
      }
      else if (!subscribed && realtimeROS2Node == null)
      {
         unsubscribe();
      }
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
   }

   public boolean isSubscribed()
   {
      return subscribed.get();
   }

   public void setPointSizeScale(float pointSizeScale)
   {
      this.pointSizeScale.set(pointSizeScale);
   }

   public void setLevelOfColorDetail(int levelOfColorDetail)
   {
      this.levelOfColorDetail.set(levelOfColorDetail);
   }

   public ImBoolean useSensorColor()
   {
      return useSensorColor;
   }

   public ImBoolean useSinusoidalGradientPattern()
   {
      return useSinusoidalGradientPattern;
   }

   public RDXColorGradientMode getGradientMode()
   {
      return gradientMode;
   }

   public void setGradientMode(RDXColorGradientMode mode)
   {
      gradientMode = mode;
   }

   public ImFloat getPointSizeScale()
   {
      return pointSizeScale;
   }

   public CameraModel getColorChannelCamera()
   {
      return colorChannel.getCameraModel();
   }

   public CameraModel getDepthChannelCamera()
   {
      return depthChannel.getCameraModel();
   }

   public ImInt getLevelOfColorDetail()
   {
      return levelOfColorDetail;
   }
}