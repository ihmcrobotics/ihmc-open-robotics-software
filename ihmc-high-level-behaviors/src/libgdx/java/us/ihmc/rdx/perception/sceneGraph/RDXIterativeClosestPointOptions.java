package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import perception_msgs.msg.dds.DetectedObjectPacket;
import perception_msgs.msg.dds.IterativeClosestPointRequest;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.RestartableThrottledThread;

public class RDXIterativeClosestPointOptions implements RenderableProvider
{
   private static final double ICP_REQUEST_FREQUENCY = 10.0;
   private static final int ICP_MAX_POINTS = 5000;

   private final RDXPrimitiveRigidBodySceneNode requestingNode;
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final IHMCROS2Publisher<IterativeClosestPointRequest> requestPublisher;
   private final IHMCROS2Input<DetectedObjectPacket> resultSubscription;
   private final RestartableThrottledThread updateThread;

   private final ImGuiUniqueLabelMap labels;
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean showICPParameters = false;
   private final ImBoolean runICP = new ImBoolean(false);
   private final ImBoolean useICPPose = new ImBoolean(false);
   private final ImBoolean showICPPointCloud = new ImBoolean(false);
   private final ImInt numberOfShapeSamples = new ImInt(750);
   private final ImInt numberOfCorrespondences = new ImInt(750);
   private final ImInt numberOfIterations = new ImInt(2);
   private final ImFloat segmentationRadius = new ImFloat(0.05f);
   private final Timer useICPPoseTimer = new Timer();

   private final RecyclingArrayList<Point3D32> icpObjectPointCloud = new RecyclingArrayList<>(ICP_MAX_POINTS, Point3D32::new);
   private final RDXPointCloudRenderer objectPointCloudRenderer = new RDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> icpSegmentedPointCloud = new RecyclingArrayList<>(ICP_MAX_POINTS, Point3D32::new);
   private final RDXPointCloudRenderer segmentationRenderer = new RDXPointCloudRenderer();
   private final RDXReferenceFrameGraphic icpFrameGraphic = new RDXReferenceFrameGraphic(0.2);

   public RDXIterativeClosestPointOptions(RDXPrimitiveRigidBodySceneNode requestingNode, ImGuiUniqueLabelMap labels)
   {
      this.requestingNode = requestingNode;
      this.labels = labels;
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "primitive_scene_node_" + requestingNode.getSceneNode().getID());
      ros2Helper = new ROS2Helper(ros2Node);
      requestPublisher = new IHMCROS2Publisher<>(ros2Node, PerceptionAPI.ICP_REQUEST);
      resultSubscription = ros2Helper.subscribe(PerceptionAPI.ICP_RESULT, message -> message.getId() == requestingNode.getSceneNode().getID());
      objectPointCloudRenderer.create(ICP_MAX_POINTS);
      segmentationRenderer.create(ICP_MAX_POINTS);

      updateThread = new RestartableThrottledThread(requestingNode.getClass().getName() + requestingNode.getSceneNode().getID() + "ICPRequest",
                                                    ICP_REQUEST_FREQUENCY,
                                                    this::update);
   }

   private void update()
   {
      IterativeClosestPointRequest requestMessage = new IterativeClosestPointRequest();
      requestMessage.setNodeId(requestingNode.getSceneNode().getID());
      requestMessage.setShape(((PrimitiveRigidBodySceneNode) requestingNode.getSceneNode()).getShape().toByte());
      requestMessage.getLengths().set(requestingNode.getLengths());
      requestMessage.getRadii().set(requestingNode.getRadii());
      requestMessage.getProvidedPose().set(requestingNode.getSceneNode().getNodeFrame().getTransformToWorldFrame());
      requestMessage.setNumberOfCorrespondences(numberOfCorrespondences.get());
      requestMessage.setNumberOfShapeSamples(numberOfShapeSamples.get());
      requestMessage.setNumberOfIterations(numberOfIterations.get());
      requestMessage.setSegmentationRadius(segmentationRadius.get());
      requestMessage.setRunIcp(runICP.get());
      requestMessage.setUseProvidedPose(!useICPPose.get());
      requestPublisher.publish(requestMessage);

      icpObjectPointCloud.clear();
      icpSegmentedPointCloud.clear();
      if (runICP.get() && resultSubscription.hasReceivedFirstMessage())
      {
         icpFrameGraphic.setPoseInWorldFrame(resultSubscription.getLatest().getPose());

         if (showICPPointCloud.get())
         {
            for (Point3D32 point : resultSubscription.getLatest().getObjectPointCloud())
               icpObjectPointCloud.add().set(point);

            for (Point3D32 point : resultSubscription.getLatest().getSegmentedPointCloud())
               icpSegmentedPointCloud.add().set(point);
         }
      }
      objectPointCloudRenderer.setPointsToRender(icpObjectPointCloud, Color.GOLD);
      objectPointCloudRenderer.updateMesh();
      segmentationRenderer.setPointsToRender(icpSegmentedPointCloud, Color.LIGHT_GRAY);
      segmentationRenderer.updateMesh();
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Run ICP"), runICP))
      {
         if (runICP.get())
         {
            runICP();
         }
         else
         {
            stopICP();
         }
      }

      // Start using ICP pose after 2.5 seconds
      if (runICP.get() && useICPPoseTimer.getElapsedTime() > 2.5 && useICPPoseTimer.getElapsedTime() < 3.0)
         useICPPose.set(true);

      if (expandCollapseRenderer.render(showICPParameters))
      {
         showICPParameters = !showICPParameters;
      }
      ImGui.sameLine();
      ImGui.text("ICP Parameters");
      if (showICPParameters)
      {
         ImGui.beginDisabled(!runICP.get());

         ImGui.checkbox(labels.get("Use ICP Pose"), useICPPose);

         ImGui.sameLine();
         ImGui.checkbox(labels.get("Show ICP Point Cloud"), showICPPointCloud);

         ImGui.endDisabled();

         if (ImGuiTools.volatileInputInt(labels.get("Num Shape Samples"), numberOfShapeSamples))
            numberOfShapeSamples.set(MathTools.clamp(numberOfShapeSamples.get(), 1, ICP_MAX_POINTS));
         if (ImGuiTools.volatileInputInt(labels.get("Num Correspondences"), numberOfCorrespondences))
            numberOfCorrespondences.set(MathTools.clamp(numberOfCorrespondences.get(), 1, ICP_MAX_POINTS));
         if (ImGuiTools.volatileInputInt(labels.get("Num Iterations"), numberOfIterations))
            numberOfIterations.set(MathTools.clamp(numberOfIterations.get(), 1, 10));
         if (ImGuiTools.volatileInputFloat(labels.get("Segmentation Radius"), segmentationRadius))
            segmentationRadius.set((float) MathTools.clamp(segmentationRadius.get(), 0.0f, Float.MAX_VALUE));
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (resultSubscription.hasReceivedFirstMessage())
      {
         objectPointCloudRenderer.getRenderables(renderables, pool);
         segmentationRenderer.getRenderables(renderables, pool);
         icpFrameGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      updateThread.blockingStop();

      stopICP();

      // send message to ICP manager to remove this node
      update();

      objectPointCloudRenderer.dispose();
      segmentationRenderer.dispose();
      icpFrameGraphic.dispose();

      requestPublisher.destroy();
      ros2Node.destroy();
   }

   public void runICP()
   {
      runICP.set(true);
      showICPPointCloud.set(true);
      updateThread.start();
      useICPPoseTimer.reset();
   }

   public void stopICP()
   {
      runICP.set(false);
      showICPPointCloud.set(false);
      useICPPose.set(false);
      updateThread.stop();
   }
}
