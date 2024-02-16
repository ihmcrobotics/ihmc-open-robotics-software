package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.YOLOv8IterativeClosestPointNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImPlotDoublePlot;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;

import java.util.Set;

public class RDXYOLOv8IterativeClosestPointNode extends RDXDetectableSceneNode
{
   private static final int ICP_MAX_POINTS = 2000;

   private final YOLOv8IterativeClosestPointNode yoloSceneNode;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final SwapReference<DetectedObjectPacket> icpResultSwapReference;
   private final Notification receivedResultNotification = new Notification();
   private final RDX3DPanel panel;

   private final RDXPointCloudRenderer icpResultRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer yoloSegmentRenderer = new RDXPointCloudRenderer();
   private final RDX3DSituatedText text;
   private final FramePose3D textPose = new FramePose3D();

   private final ImGuiUniqueLabelMap labels;
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean showParameters = false;
   private final ImBoolean runICP;
   private final ImInt icpIterations;
   private final ImInt maskErosionKernelRadius;
   private final ImDouble outlierFilterThreshold;
   private final ImDouble movementDistanceThreshold;
   private final ImPlotDoublePlot detectionFrequencyPlot = new ImPlotDoublePlot("Detection Frequency", 20);
   private final ImGuiPlot movementDistanceThresholdPlot = new ImGuiPlot("Distance Threshold", 1000, 0, 20);

   public RDXYOLOv8IterativeClosestPointNode(YOLOv8IterativeClosestPointNode yoloSceneNode, RDX3DPanel panel, ImGuiUniqueLabelMap labels)
   {
      super(yoloSceneNode);

      this.yoloSceneNode = yoloSceneNode;
      this.labels = labels;
      this.panel = panel;

      runICP = new ImBoolean(yoloSceneNode.isRunningICP());
      icpIterations = new ImInt(yoloSceneNode.getICPIterations());
      maskErosionKernelRadius = new ImInt(yoloSceneNode.getMaskErosionKernelRadius());
      outlierFilterThreshold = new ImDouble(yoloSceneNode.getOutlierFilterThreshold());
      movementDistanceThreshold = new ImDouble(yoloSceneNode.getBaseDistanceThreshold());

      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "yolo_icp_scene_node_" + getSceneNode().getID());
      ros2Helper = new ROS2Helper(ros2Node);

      icpResultSwapReference = ros2Helper.subscribeViaSwapReference(PerceptionAPI.ICP_RESULT, receivedResultNotification);
      icpResultRenderer.create(ICP_MAX_POINTS);
      yoloSegmentRenderer.create(ICP_MAX_POINTS);
      text = new RDX3DSituatedText(getSceneNode().getName(), 0.05f);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      super.update(modificationQueue);

      yoloSceneNode.setRunICP(runICP.get());
      yoloSceneNode.setICPIterations(icpIterations.get());
      yoloSceneNode.setMaskErosionKernelRadius(maskErosionKernelRadius.get());
      yoloSceneNode.setOutlierFilterThreshold(outlierFilterThreshold.get());
      yoloSceneNode.setBaseDistanceThreshold(movementDistanceThreshold.get());

      textPose.setToZero(panel.getCamera3D().getCameraFrame());
      textPose.getOrientation().appendPitchRotation(3.0 / 2.0 * Math.PI);
      textPose.getOrientation().appendYawRotation(-Math.PI / 2.0);

      textPose.changeFrame(ReferenceFrame.getWorldFrame());
      textPose.getPosition().set(yoloSceneNode.getNodeFrame().getTransformToWorldFrame().getTranslation());
      textPose.getPosition().add(0.05, 0.0, 0.05);
      LibGDXTools.toLibGDX(textPose, new RigidBodyTransform(), text.getModelTransform());
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      detectionFrequencyPlot.addValue(yoloSceneNode.getDetectionFrequency());
      detectionFrequencyPlot.renderImGuiWidgets();
      movementDistanceThresholdPlot.render(yoloSceneNode.getMovementDistanceThreshold());

      if (expandCollapseRenderer.render(showParameters))
      {
         showParameters = !showParameters;
      }
      ImGui.sameLine();
      ImGui.text("Parameters");

      if (showParameters)
      {
         ImGui.checkbox(labels.get("Run ICP"), runICP);
         if (ImGuiTools.volatileInputInt(labels.get("Number of ICP Iterations"), icpIterations))
            icpIterations.set(MathTools.clamp(icpIterations.get(), 1, 10));
         if (ImGuiTools.volatileInputInt(labels.get("Mask Erosion Kernel Radius"), maskErosionKernelRadius))
            maskErosionKernelRadius.set(MathTools.clamp(maskErosionKernelRadius.get(), 0, 10));
         if (ImGuiTools.volatileInputDouble(labels.get("Outlier Filter Threshold"), outlierFilterThreshold))
            outlierFilterThreshold.set(MathTools.clamp(outlierFilterThreshold.get(), 0.0, 5.0));
         if (ImGuiTools.volatileInputDouble(labels.get("Movement Distance Threshold"), movementDistanceThreshold))
            movementDistanceThreshold.set(MathTools.clamp(movementDistanceThreshold.get(), 1000.0, 5000.0));
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      synchronized (icpResultSwapReference)
      {
         if (receivedResultNotification.poll() && icpResultSwapReference.getForThreadTwo().getId() == getSceneNode().getID())
         {
            DetectedObjectPacket resultMessage = icpResultSwapReference.getForThreadTwo();

            icpResultRenderer.setPointsToRender(resultMessage.getObjectPointCloud(), Color.GOLD);
            icpResultRenderer.updateMesh();

            yoloSegmentRenderer.setPointsToRender(resultMessage.getSegmentedPointCloud(), Color.GRAY);
            yoloSegmentRenderer.updateMesh();
         }
      }

      icpResultRenderer.getRenderables(renderables, pool);
      yoloSegmentRenderer.getRenderables(renderables, pool);
      text.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();
      ros2Node.destroy();
   }
}
