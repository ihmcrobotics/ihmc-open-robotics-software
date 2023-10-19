package us.ihmc.avatar.colorVision;

import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.ros2.ROS2Topic;

public class CenterposeSceneGraphOnRobotProcess
{
   private final IHMCROS2Input<DetectedObjectPacket> subscriber;

   public CenterposeSceneGraphOnRobotProcess(ROS2Helper ros2Helper)
   {
      ROS2Topic<DetectedObjectPacket> topicName = PerceptionAPI.CENTERPOSE_DETECTED_OBJECT;
      subscriber = ros2Helper.subscribe(topicName);
   }

   public void update(ROS2SceneGraph onRobotSceneGraph, ReferenceFrame sensorFrame)
   {
      onRobotSceneGraph.updateSubscription();
      updateSceneGraph(onRobotSceneGraph, sensorFrame);
      onRobotSceneGraph.updateOnRobotOnly(sensorFrame);
      onRobotSceneGraph.updatePublication();
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph, ReferenceFrame cameraFrame)
   {
      // Handle a new DetectedObjectPacket message
      if (subscriber.getMessageNotification().poll())
      {
         DetectedObjectPacket detectedObjectPacket = subscriber.getMessageNotification().read();

         // Update or add the corresponding CenterposeSceneNode
         sceneGraph.modifyTree(modificationQueue ->
         {
            final CenterposeNode centerposeNode;

            Point3D[] vertices = detectedObjectPacket.getBoundingBoxVertices();
            for (Point3D vertex : vertices)
            {
               FramePoint3D frameVertex = new FramePoint3D();
               frameVertex.setIncludingFrame(cameraFrame, vertex);
               frameVertex.changeFrame(ReferenceFrame.getWorldFrame());
               vertex.set(frameVertex);
            }

            // Update
            if (sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().containsKey(detectedObjectPacket.getId()))
            {
               centerposeNode = sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().get(detectedObjectPacket.getId());

               centerposeNode.setSequenceID(detectedObjectPacket.getSequenceId());
               centerposeNode.setVertices3D(vertices);
               centerposeNode.setObjectType(detectedObjectPacket.getObjectTypeAsString());
               centerposeNode.setConfidence(detectedObjectPacket.getConfidence());
            }
            // Add
            else
            {
               centerposeNode = new CenterposeNode(sceneGraph.getNextID().getAndIncrement(),
                                                   "CenterposeDetectedObject%d".formatted(detectedObjectPacket.getId()),
                                                   detectedObjectPacket.getId(),
                                                   vertices,
                                                   detectedObjectPacket.getBoundingBox2dVertices());

               modificationQueue.accept(new SceneGraphNodeAddition(centerposeNode, sceneGraph.getRootNode()));
               sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().put(centerposeNode.getObjectID(), centerposeNode);
               sceneGraph.getDetectionFilterCollection().createFilter(centerposeNode.getObjectID(), 10);
            }

            centerposeNode.applyFilter();
            centerposeNode.getNodeFrame().update();

            sceneGraph.getDetectionFilterCollection().getFilter(centerposeNode.getObjectID()).registerDetection();
         });
      }

      // Update the detected status for all CenterposeNode\s in the SceneGraph
      for (CenterposeNode centerposeNode : sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().valueCollection())
      {
         boolean currentlyDetected = sceneGraph.getDetectionFilterCollection().getFilter(centerposeNode.getObjectID()).isStableDetectionResult();
         centerposeNode.setCurrentlyDetected(currentlyDetected);
      }
   }

   public void destroy()
   {
      subscriber.destroy();
   }
}
