package us.ihmc.perception.sceneGraph.centerpose;

import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.filters.TimeBasedDetectionFilter;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Topic;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class CenterposeDetectionManager
{
   public static final RigidBodyTransform CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM = new RigidBodyTransform();
   static
   {
      CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM.getRotation().setEuler(0.0, Math.toRadians(90.0), Math.toRadians(180.0));
   }

   private final Map<Integer, TimeBasedDetectionFilter> centerposeNodeDetectionFilters = new HashMap<>();
   private final ReferenceFrame centerposeOutputFrame;
   private final MutableReferenceFrame imageAquisitionSensorFrame = new MutableReferenceFrame();

   private final ConcurrentLinkedQueue<DetectedObjectPacket> messageQueue = new ConcurrentLinkedQueue<>();

   public CenterposeDetectionManager(ROS2Helper ros2Helper)
   {
      ROS2Topic<DetectedObjectPacket> topicName = PerceptionAPI.CENTERPOSE_DETECTED_OBJECT;
      ros2Helper.subscribeViaCallback(topicName, messageQueue::add);

      centerposeOutputFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("CenterposeOutputFrame",
                                                                                                imageAquisitionSensorFrame.getReferenceFrame(),
                                                                                                CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM);
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      // Handle a new DetectedObjectPacket message
      while (!messageQueue.isEmpty())
      {
         DetectedObjectPacket detectedObjectPacket = messageQueue.remove();

         imageAquisitionSensorFrame.getTransformToParent().set(detectedObjectPacket.getSensorPose());
         imageAquisitionSensorFrame.getReferenceFrame().update();

         // Update or add the corresponding CenterposeSceneNode
         sceneGraph.modifyTree(modificationQueue ->
         {
          CenterposeNode centerposeNode;

            Point3D[] vertices = detectedObjectPacket.getBoundingBoxVertices();
            for (Point3D vertex : vertices)
            {
               FramePoint3D frameVertex = new FramePoint3D();
               frameVertex.setIncludingFrame(centerposeOutputFrame, vertex);
               frameVertex.changeFrame(ReferenceFrame.getWorldFrame());
               vertex.set(frameVertex);
            }

            // Update
            if (sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().containsKey(detectedObjectPacket.getId()))
            {
               centerposeNode = sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().get(detectedObjectPacket.getId());

               centerposeNode.setVertices3D(vertices);
               centerposeNode.setObjectType(detectedObjectPacket.getObjectTypeAsString());
               centerposeNode.setConfidence(detectedObjectPacket.getConfidence());

               Pose3D objectPoseInSensorFrame = detectedObjectPacket.getPose();
               FramePose3D objectOriginPoseFrame = new FramePose3D();
               objectOriginPoseFrame.setIncludingFrame(centerposeOutputFrame, objectPoseInSensorFrame);
               objectOriginPoseFrame.changeFrame(ReferenceFrame.getWorldFrame());
               centerposeNode.getNodeToParentFrameTransform().set(objectOriginPoseFrame);
            }
            // Add
            else
            {
               centerposeNode = new CenterposeNode(sceneGraph.getNextID().getAndIncrement(),
                                                   "CenterposeDetectedObject%d".formatted(detectedObjectPacket.getId()),
                                                   detectedObjectPacket.getId(),
                                                   vertices,
                                                   detectedObjectPacket.getBoundingBox2dVertices(),
                                                   true);

               modificationQueue.accept(new SceneGraphNodeAddition(centerposeNode, sceneGraph.getRootNode()));
               sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().put(centerposeNode.getObjectID(), centerposeNode);
               centerposeNodeDetectionFilters.put(centerposeNode.getObjectID(), new TimeBasedDetectionFilter(1.0f, 2));
            }

            centerposeNodeDetectionFilters.get(centerposeNode.getObjectID()).registerDetection();
         });
      }

      for (CenterposeNode centerposeNode : sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().valueCollection())
      {
         centerposeNode.update();
         centerposeNode.setCurrentlyDetected(centerposeNodeDetectionFilters.get(centerposeNode.getObjectID()).isDetected());
      }
   }

   public RigidBodyTransform getImageAquisitionSensorFrameTransformToRoot()
   {
      return imageAquisitionSensorFrame.getReferenceFrame().getTransformToRoot();
   }

   public void destroy()
   {
      messageQueue.clear();
   }
}
