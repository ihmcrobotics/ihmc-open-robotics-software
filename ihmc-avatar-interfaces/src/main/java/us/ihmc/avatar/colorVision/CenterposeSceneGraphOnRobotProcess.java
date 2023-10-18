package us.ihmc.avatar.colorVision;

import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.perception.filters.DetectionFilter;

public class CenterposeSceneGraphOnRobotProcess
{
   private final IHMCROS2Input<DetectedObjectPacket> subscriber;
   private DetectedObjectPacket detectedObjectMessage;
   private final ROS2Topic<DetectedObjectPacket> topicName = PerceptionAPI.CENTERPOSE_DETECTED_OBJECT;
   private final ReferenceFrame sensorInZEDFrame;
   private final FramePose3D markerPose = new FramePose3D();
   private final FramePoint3D[] verticesFramePoint3D = new FramePoint3D[8];

   public CenterposeSceneGraphOnRobotProcess(ROS2Helper ros2Helper)
   {
      subscriber = ros2Helper.subscribe(topicName);

      RigidBodyTransform sensorInWorldTransform = new RigidBodyTransform();
      sensorInWorldTransform.getTranslation().set(0.0, 0.06, 0.0);
      sensorInWorldTransform.getRotation().setEuler(0.0, Math.toRadians(90.0), Math.toRadians(180.0));
      sensorInZEDFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("SensorFrame",
                                                                                      ReferenceFrame.getWorldFrame(),
                                                                                      sensorInWorldTransform);
   }

   public void update(ROS2SceneGraph onRobotSceneGraph, ReferenceFrame sensorFrame)
   {
      onRobotSceneGraph.updateSubscription();
      this.updateSceneGraph(onRobotSceneGraph);
      onRobotSceneGraph.updateOnRobotOnly(sensorFrame);      // Is it the correct frame?
      onRobotSceneGraph.updatePublication();
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
                                            DetectionFilter candidateFilter = sceneGraph.getDetectionFilterCollection().getFilter(detectedID);
                                            if (candidateFilter==null)
                                               candidateFilter = sceneGraph.getDetectionFilterCollection().createFilter(detectedID, 10);
                                            candidateFilter.registerDetection();
                                            if (candidateFilter.isStableDetectionResult())
                                            {
                                               sceneGraph.getDetectionFilterCollection().removeFilter(detectedID);

                                               Point3D[] vertices = detectedObjectMessage.getBoundingBoxVertices();
                                               for (int i = 0; i < vertices.length; i++)
                                               {
                                                  if (verticesFramePoint3D[i] == null)
                                                     verticesFramePoint3D[i] = new FramePoint3D();
                                                  verticesFramePoint3D[i].changeFrame(sensorInZEDFrame);
                                                  verticesFramePoint3D[i].interpolate(vertices[i], 0.2);
                                                  verticesFramePoint3D[i].changeFrame(ReferenceFrame.getWorldFrame());
                                                  vertices[i].set(verticesFramePoint3D[i].getX(), verticesFramePoint3D[i].getY(), verticesFramePoint3D[i].getZ());
                                               }

                                               String nodeName = "CenterposeDetectedObject%d".formatted(detectedID);
                                               CenterposeDetectedMarkerNode = new CenterposeNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                 nodeName,
                                                                                                 detectedID,
                                                                                                 vertices,
                                                                                                 detectedObjectMessage.getBoundingBox2dVertices());
                                               LogTools.info("Adding detected Centerpose Detected marker {} to scene graph as {}", detectedID, nodeName);
                                               modificationQueue.accept(new SceneGraphNodeAddition(CenterposeDetectedMarkerNode, sceneGraph.getRootNode()));
                                               sceneGraph.getCenterposeDetectedMarkerIDToNodeMap().put(detectedID, CenterposeDetectedMarkerNode); // Prevent it getting added twice
                                            }
                                         }
                                      });

         // All Centerpose markers are child of root
         // This must be done after the above are added to the scene graph
         for (SceneNode child : sceneGraph.getRootNode().getChildren())
         {
            if (child instanceof CenterposeNode centerposeDetectedMarkerNode)
            {
               boolean isDetected = detectedObjectMessage.getSequenceId() == centerposeDetectedMarkerNode.getSequenceID() + 1 ;
               centerposeDetectedMarkerNode.setCurrentlyDetected(isDetected);
               if (isDetected)
               {
                  centerposeDetectedMarkerNode.setSequenceID(detectedObjectMessage.getSequenceId());
                  Point3D[] vertices = detectedObjectMessage.getBoundingBoxVertices();
                  for (int i = 0; i < vertices.length; i++)
                  {
                     if (verticesFramePoint3D[i] == null)
                        verticesFramePoint3D[i] = new FramePoint3D();
                     verticesFramePoint3D[i].changeFrame(sensorInZEDFrame);
                     verticesFramePoint3D[i].interpolate(vertices[i], 0.2);
                     verticesFramePoint3D[i].changeFrame(ReferenceFrame.getWorldFrame());
                     vertices[i].set(verticesFramePoint3D[i].getX(), verticesFramePoint3D[i].getY(), verticesFramePoint3D[i].getZ());
                  }
                  centerposeDetectedMarkerNode.setVertices3D(vertices);

                  Pose3D objectPoseSensorFrame = detectedObjectMessage.getPose();
                  markerPose.setIncludingFrame(sensorInZEDFrame, objectPoseSensorFrame);
                  markerPose.changeFrame(ReferenceFrame.getWorldFrame());
                  centerposeDetectedMarkerNode.getNodeToParentFrameTransform().set(markerPose);

                  centerposeDetectedMarkerNode.setConfidence(detectedObjectMessage.getConfidence());

                  centerposeDetectedMarkerNode.setObject_type(detectedObjectMessage.getObjectTypeAsString());

                  centerposeDetectedMarkerNode.applyFilter();
                  centerposeDetectedMarkerNode.getNodeFrame().update();
               }
            }
         }
      }
   }

   public void destroy()
   {
      subscriber.destroy();
   }
}
