package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;

import java.util.Set;

public class RDXYoloDetectableSceneNode extends RDXDetectableSceneNode
{
   private static final int ICP_MAX_POINTS = 2000;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final SwapReference<DetectedObjectPacket> icpResultSwapReference;
   private final Notification receivedResultNotification = new Notification();

   private final RDXPointCloudRenderer icpResultRenderer = new RDXPointCloudRenderer();

   public RDXYoloDetectableSceneNode(DetectableSceneNode detectableSceneNode)
   {
      super(detectableSceneNode);

      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "yolo_icp_scene_node_" + getSceneNode().getID());
      ros2Helper = new ROS2Helper(ros2Node);

      icpResultSwapReference = ros2Helper.subscribeViaSwapReference(PerceptionAPI.ICP_RESULT, receivedResultNotification);
      icpResultRenderer.create(ICP_MAX_POINTS);
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
         }
      }

      icpResultRenderer.getRenderables(renderables, pool);
   }

   @Override
   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.remove(modificationQueue, sceneGraph);
      ros2Node.destroy();
   }
}
