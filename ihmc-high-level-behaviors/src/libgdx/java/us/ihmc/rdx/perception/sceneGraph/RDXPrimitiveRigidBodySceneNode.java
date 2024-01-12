package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.Set;

/**
 * A "ghost" colored model.
 */
public class RDXPrimitiveRigidBodySceneNode extends RDXRigidBodySceneNode
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private static final float DEFAULT_DIMENSION = 0.1F;
   private static final double ICP_REQUEST_FREQUENCY = 5.0;
   private static final int ICP_MAX_POINTS = 10000;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "primitive_scene_node_" + getSceneNode().getID());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private final IHMCROS2Publisher<IterativeClosestPointRequest> requestPublisher = new IHMCROS2Publisher<>(ros2Node, PerceptionAPI.ICP_REQUEST);
   private final IHMCROS2Input<DetectedObjectPacket> icpResultSubscription;
   private final RestartableThrottledThread requestPublisherThread;

   private RDXModelInstance modelInstance;

   private final ImFloat xLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat xRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zRadius = new ImFloat(DEFAULT_DIMENSION);

   private final ImBoolean runICP = new ImBoolean(false);
   private final ImBoolean useICPPose = new ImBoolean(false);
   private final ImBoolean showICPPointCloud = new ImBoolean(false);

   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean showICPParameters = false;
   private final ImInt numberOfShapeSamples = new ImInt(1000);
   private final ImInt numberOfCorrespondences = new ImInt(1000);
   private final ImInt numberOfIterations = new ImInt(1);
   private final ImFloat segmentationRadius = new ImFloat(0.2f);

   private final RecyclingArrayList<Point3D32> icpObjectPointCloud = new RecyclingArrayList<>(32768, Point3D32::new);
   private final RDXPointCloudRenderer icpObjectPointCloudRenderer = new RDXPointCloudRenderer();
   private boolean updateObjectPointCloudMesh;
   private final RDXReferenceFrameGraphic icpFrameGraphic = new RDXReferenceFrameGraphic(0.2);

   public RDXPrimitiveRigidBodySceneNode(PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode, RDX3DPanel panel3D)
   {
      this(new Vector3D32(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION),
           new Vector3D32(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION),
           primitiveRigidBodySceneNode,
           panel3D);
   }

   public RDXPrimitiveRigidBodySceneNode(Vector3D32 lengths, Vector3D32 radii, PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode, RDX3DPanel panel3D)
   {
      super(primitiveRigidBodySceneNode, new RigidBodyTransform(), panel3D);

      if (lengths == null)
         lengths = new Vector3D32(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION);
      if (radii == null)
         radii = new Vector3D32(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION);

      xLength.set(lengths.getX32());
      yLength.set(lengths.getY32());
      zLength.set(lengths.getZ32());
      xRadius.set(radii.getX32());
      yRadius.set(radii.getY32());
      zRadius.set(radii.getZ32());

      switch (primitiveRigidBodySceneNode.getShape())
      {
         case BOX -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(lengths.getX32(), lengths.getY32(), lengths.getZ32(), Color.WHITE));
         case PRISM -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(lengths.getX32(), lengths.getY32(), lengths.getZ32(), Color.WHITE));
         case CYLINDER -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(lengths.getZ32(), radii.getX32(), Color.WHITE));
         case ELLIPSOID -> modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(radii.getX32(), radii.getY32(), radii.getZ32(), Color.WHITE));
         case CONE -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(lengths.getZ32(), radii.getX32(), Color.WHITE));
      }
      modelInstance.setColor(GHOST_COLOR);

      requestPublisherThread = new RestartableThrottledThread(getClass().getName() + getSceneNode().getID() + "ICPRequest",
                                                              ICP_REQUEST_FREQUENCY,
                                                              this::updateICP);
      icpResultSubscription = ros2Helper.subscribe(PerceptionAPI.ICP_RESULT, message -> message.getId() == getSceneNode().getID());

      icpObjectPointCloudRenderer.create(100000);
      panel3D.getScene().addRenderableProvider(icpObjectPointCloudRenderer, RDXSceneLevel.VIRTUAL);
   }

   private void updateICP()
   {
      IterativeClosestPointRequest requestMessage = new IterativeClosestPointRequest();
      requestMessage.setNodeId(getSceneNode().getID());
      requestMessage.setShape(((PrimitiveRigidBodySceneNode) getSceneNode()).getShape().toByte());
      requestMessage.getLengths().set(xLength.get(), yLength.get(), zLength.get());
      requestMessage.getRadii().set(xRadius.get(), yRadius.get(), zRadius.get());
      requestMessage.getProvidedPose().set(getSceneNode().getNodeFrame().getTransformToWorldFrame());
      requestMessage.setNumberOfCorrespondences(numberOfCorrespondences.get());
      requestMessage.setNumberOfShapeSamples(numberOfShapeSamples.get());
      requestMessage.setNumberOfIterations(numberOfIterations.get());
      requestMessage.setSegmentationRadius(segmentationRadius.get());
      requestMessage.setRunIcp(runICP.get());
      requestMessage.setUseProvidedPose(!useICPPose.get());
      requestPublisher.publish(requestMessage);

      icpObjectPointCloud.clear();
      if (runICP.get() && icpResultSubscription.hasReceivedFirstMessage())
      {
         if (icpResultSubscription.getMessageNotification().poll())
            System.out.println("Received update: " + icpResultSubscription.getLatest().getPose());

         icpFrameGraphic.setPoseInWorldFrame(icpResultSubscription.getLatest().getPose());
         getModelInstance().setPoseInWorldFrame(icpResultSubscription.getLatest().getPose());

         if (showICPPointCloud.get())
         {
            for (Point3D32 point3D32 : icpResultSubscription.getLatest().getObjectPointCloud())
               icpObjectPointCloud.add().set(point3D32);
         }
         updateObjectPointCloudMesh = true;
      }
      icpObjectPointCloudRenderer.setPointsToRender(icpObjectPointCloud, Color.GOLD);

   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      if (ImGui.checkbox(labels.get("Run ICP"), runICP))
      {
         if (runICP.get())
            requestPublisherThread.start();
         else
            requestPublisherThread.stop();
      }

      ImGui.beginDisabled(!runICP.get());

      ImGui.sameLine();
      ImGui.checkbox(labels.get("Use ICP Pose"), useICPPose);

      ImGui.endDisabled();

      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show ICP Point Cloud"), showICPPointCloud);

      if (expandCollapseRenderer.render(showICPParameters))
      {
         showICPParameters = !showICPParameters;
      }
      ImGui.sameLine();
      ImGui.text("ICP Parameters");
      if (showICPParameters)
      {
         if (ImGuiTools.volatileInputInt(labels.get("Num Shape Samples"), numberOfShapeSamples))
            numberOfShapeSamples.set(MathTools.clamp(numberOfShapeSamples.get(), 1, ICP_MAX_POINTS));
         if (ImGuiTools.volatileInputInt(labels.get("Num Correspondences"), numberOfCorrespondences))
            numberOfCorrespondences.set(MathTools.clamp(numberOfCorrespondences.get(), 1, ICP_MAX_POINTS));
         if (ImGuiTools.volatileInputInt(labels.get("Num Iterations"), numberOfIterations))
            numberOfIterations.set(MathTools.clamp(numberOfIterations.get(), 1, 10));
         if (ImGuiTools.volatileInputFloat(labels.get("Segmentation Radius"), segmentationRadius))
            segmentationRadius.set((float) MathTools.clamp(segmentationRadius.get(), 0.0f, Float.MAX_VALUE));
      }

      ImGui.text("Modify shape:");

      PrimitiveRigidBodyShape shape = ((PrimitiveRigidBodySceneNode) getSceneNode()).getShape();

      switch (shape)
      {
         case BOX ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("depth"), xLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("width"), yLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(xLength.get(), yLength.get(), zLength.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case PRISM ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("depth"), xLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("width"), yLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(xLength.get(), yLength.get(), zLength.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case CYLINDER ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("radius"), xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(zLength.get(), xRadius.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case ELLIPSOID ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("xRadius"), xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("yRadius"), yRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("zRadius"), zRadius))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(xRadius.get(), yRadius.get(), zRadius.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case CONE ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat(labels.get("radius"), xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat(labels.get("height"), zLength))
               reshaped = true;
            if (reshaped)
            {
               if (modelInstance != null)
                  modelInstance.model.dispose();
               modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(zLength.get(), xRadius.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         modelInstance.getRenderables(renderables, pool);

      if (icpResultSubscription.hasReceivedFirstMessage())
      {
         if (updateObjectPointCloudMesh)
            icpObjectPointCloudRenderer.updateMesh();
         icpFrameGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   @Override
   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.remove(modificationQueue, sceneGraph);
      requestPublisherThread.blockingStop();

      showICPPointCloud.set(false);
      useICPPose.set(false);
      runICP.set(false);

      // send message to ICP manager to remove this node
      updateICP();

      requestPublisher.destroy();
      ros2Node.destroy();
   }
}
