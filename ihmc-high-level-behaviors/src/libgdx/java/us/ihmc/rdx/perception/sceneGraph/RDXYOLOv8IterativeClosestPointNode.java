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
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.YOLOv8IterativeClosestPointNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;

import java.util.Set;

public class RDXYOLOv8IterativeClosestPointNode extends RDXDetectableSceneNode
{
   private static final int ICP_MAX_POINTS = 2000;

   private YOLOv8IterativeClosestPointNode yoloSceneNode;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final SwapReference<DetectedObjectPacket> icpResultSwapReference;
   private final Notification receivedResultNotification = new Notification();

   private final RDXPointCloudRenderer icpResultRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer yoloSegmentRenderer = new RDXPointCloudRenderer();

   private final ImGuiUniqueLabelMap labels;
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean showParameters = false;
   private final ImBoolean runICP;
   private final ImInt icpIterations;
   private final ImInt maskErosionKernelRadius;
   private final ImDouble outlierFilterThreshold;
   private final ImDouble movementDistanceThreshold;

   public RDXYOLOv8IterativeClosestPointNode(YOLOv8IterativeClosestPointNode yoloSceneNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloSceneNode);

      this.yoloSceneNode = yoloSceneNode;
      this.labels = labels;

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
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      ImGui.text("Distance Threshold: " + yoloSceneNode.getMovementDistanceThreshold());

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
   }

   @Override
   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.remove(modificationQueue, sceneGraph);
      ros2Node.destroy();
   }
}
