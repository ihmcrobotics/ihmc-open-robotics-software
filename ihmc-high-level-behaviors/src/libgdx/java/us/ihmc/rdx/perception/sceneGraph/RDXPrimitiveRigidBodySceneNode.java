package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import perception_msgs.msg.dds.DetectedObjectPacket;
import perception_msgs.msg.dds.IterativeClosestPointRequest;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.pubsub.DomainFactory;
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

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "primitive_scene_node_" + getSceneNode().getID());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private final IHMCROS2Publisher<IterativeClosestPointRequest> requestPublisher = new IHMCROS2Publisher<>(ros2Node, PerceptionAPI.ICP_REQUEST);
   private final IHMCROS2Input<DetectedObjectPacket> icpResultSubscription;
   private final RestartableThrottledThread requestPublisherThread;

   private RDXModelInstance modelInstance;
   private RDXReferenceFrameGraphic icpFrameGraphic = new RDXReferenceFrameGraphic(0.2);

   private final ImFloat xLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat xRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zRadius = new ImFloat(DEFAULT_DIMENSION);

   private final ImBoolean runICP = new ImBoolean(false);
   private final ImBoolean useICPPose = new ImBoolean(false);

   public RDXPrimitiveRigidBodySceneNode(PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode, RDX3DPanel panel3D)
   {
      super(primitiveRigidBodySceneNode, new RigidBodyTransform(), panel3D);

      switch (primitiveRigidBodySceneNode.getShape())
      {
         case BOX -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case PRISM -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case CYLINDER -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case ELLIPSOID ->
               modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case CONE -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
      }
      modelInstance.setColor(GHOST_COLOR);

      requestPublisherThread = new RestartableThrottledThread(getClass().getName() + getSceneNode().getID() + "ICPRequest",
                                                              ICP_REQUEST_FREQUENCY,
                                                              this::publishICPRequest);
      icpResultSubscription = ros2Helper.subscribe(PerceptionAPI.ICP_RESULT, message -> message.getId() == getSceneNode().getID());
   }

   private void publishICPRequest()
   {
      IterativeClosestPointRequest requestMessage = new IterativeClosestPointRequest();
      requestMessage.setNodeId(getSceneNode().getID());
      requestMessage.setShape(((PrimitiveRigidBodySceneNode) getSceneNode()).getShape().toByte());
      requestMessage.setXLength(xLength.get());
      requestMessage.setYLength(yLength.get());
      requestMessage.setZLength(zLength.get());
      requestMessage.setXRadius(xRadius.get());
      requestMessage.setYRadius(yRadius.get());
      requestMessage.setZRadius(zRadius.get());
      requestMessage.getProvidedPose().set(getSceneNode().getNodeFrame().getTransformToWorldFrame());
      requestMessage.setRunIcp(runICP.get());
      requestMessage.setUseProvidedPose(!useICPPose.get());
      requestPublisher.publish(requestMessage);

      if (runICP.get() && icpResultSubscription.hasReceivedFirstMessage())
      {
         icpFrameGraphic.setPoseInWorldFrame(icpResultSubscription.getLatest().getPose());
      }
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

      if (runICP.get())
      {
         ImGui.sameLine();
         ImGui.checkbox(labels.get("Use ICP Pose"), useICPPose);
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
         icpFrameGraphic.getRenderables(renderables, pool);
   }

   @Override
   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
