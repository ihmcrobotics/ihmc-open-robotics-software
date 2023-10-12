package us.ihmc.avatar.colorVision;

import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class CenterposeSceneGraphOnRobotProcess
{
   private final ROS2Node ros2Node;
   private ROS2SceneGraph onRobotSceneGraph;
   private ROS2SyncedRobotModel syncedRobot;
   private volatile boolean destroyed = false;
   private final ROS2Helper ros2Helper;
   private final IHMCROS2Input<DetectedObjectPacket> subscriber;
   private DetectedObjectPacket detectedObjectMessage;
   private final ROS2Topic<DetectedObjectPacket> topicName = PerceptionAPI.CENTERPOSE_DETECTED_OBJECT;
   private final DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;

   public CenterposeSceneGraphOnRobotProcess(DRCRobotModel robotModel, boolean simulation)
   {
      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "centerpose_scene_graph_node");

      ros2Helper = new ROS2Helper(ros2Node);
      subscriber = ros2Helper.subscribe(topicName);

      if (!simulation)
      {
         onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);
         syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
         while (!destroyed)
         {
            onRobotSceneGraph.updateSubscription();

            this.updateSceneGraph(onRobotSceneGraph);

            syncedRobot.update();
            onRobotSceneGraph.updateOnRobotOnly(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());      // Is it the correct frame?

            onRobotSceneGraph.updatePublication();
         }
      }
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      if (subscriber.getMessageNotification().poll())
      {
         detectedObjectMessage = subscriber.getMessageNotification().read();

         sceneGraph.modifyTree(modificationQueue ->
                                      {
                                         int detectedID = detectedObjectMessage.getId();
                                         CenterposeNode CenterposeDetectedMarkerNode = sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().get(detectedID);
                                         if (CenterposeDetectedMarkerNode == null) // Add node if it is missing
                                         {
                                            DetectionFilter candidateFilter = sceneGraph.getDetectionFilterCollection().getOrCreateFilter(detectedID);
                                            candidateFilter.registerDetection();
                                            if (candidateFilter.isStableDetectionResult())
                                            {
                                               sceneGraph.getDetectionFilterCollection().removeFilter(detectedID);

                                               String nodeName = "CenterposeDetectedObject%d".formatted(detectedID);
                                               CenterposeDetectedMarkerNode = new CenterposeNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                 nodeName,
                                                                                                 detectedID);
                                               LogTools.info("Adding detected Centerpose Detected marker {} to scene graph as {}", detectedID, nodeName);
                                               modificationQueue.accept(new SceneGraphNodeAddition(CenterposeDetectedMarkerNode, sceneGraph.getRootNode()));
                                               sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().put(detectedID, CenterposeDetectedMarkerNode); // Prevent it getting added twice
                                            }
                                         }
                                      });

         // All ArUco markers are child of root
         // This must be done after the above are added to the scene graph
         for (SceneNode child : sceneGraph.getRootNode().getChildren())
         {
            if (child instanceof CenterposeNode centerposeDetectedMarkerNode)
            {
               boolean isDetected = detectedObjectMessage.getId() == centerposeDetectedMarkerNode.getMarkerID();
               centerposeDetectedMarkerNode.setCurrentlyDetected(isDetected);
               if (isDetected)
               {
                  centerposeDetectedMarkerNode.getNodeToParentFrameTransform().set(detectedObjectMessage.getPose().getOrientation(), detectedObjectMessage.getPose().getPosition());
                  centerposeDetectedMarkerNode.applyFilter();
                  centerposeDetectedMarkerNode.getNodeFrame().update();
               }
            }
         }
      }
   }

   public void destroy()
   {
      destroyed = true;
      ros2Node.destroy();
   }
}
