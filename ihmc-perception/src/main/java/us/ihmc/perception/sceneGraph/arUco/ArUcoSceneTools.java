package us.ihmc.perception.sceneGraph.arUco;

import gnu.trove.iterator.TIntIterator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;

/**
 * This class exists to perform some operations that are like "glue" between the scene based
 * objects and the OpenCV based ArUco marker detection.
 */
public class ArUcoSceneTools
{
   public static void updateSceneGraph(OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults, ReferenceFrame sensorFrame, ROS2SceneGraph sceneGraph)
   {
      if (!arUcoMarkerDetectionResults.getDetectedIDs().isEmpty())
      {
         sceneGraph.modifyTree(modificationQueue ->
         {
            for (TIntIterator iterator = arUcoMarkerDetectionResults.getDetectedIDs().iterator(); iterator.hasNext(); )
            {
               int detectedID = iterator.next();
               ArUcoMarkerNode arUcoMarkerNode = sceneGraph.getArUcoMarkerIDToNodeMap().get(detectedID);
               if (arUcoMarkerNode == null) // Add ArUco marker node if it is missing
               {
                  // Make sure the marker is one we know about and get the size
                  double markerSize = RigidBodySceneObjectDefinitions.ARUCO_MARKER_SIZES.get(detectedID);
                  if (markerSize == RigidBodySceneObjectDefinitions.ARUCO_MARKER_SIZES.getNoEntryValue())
                  {
                     markerSize = DoorSceneNodeDefinitions.ARUCO_MARKER_SIZES.get(detectedID);
                  }

                  if (markerSize != DoorSceneNodeDefinitions.ARUCO_MARKER_SIZES.getNoEntryValue())
                  {
                     DetectionFilter candidateFilter = sceneGraph.getDetectionFilterCollection().getOrCreateFilter(detectedID);
                     candidateFilter.registerDetection();
                     if (candidateFilter.isStableDetectionResult())
                     {
                        sceneGraph.getDetectionFilterCollection().removeFilter(detectedID);

                        String nodeName = "ArUcoMarker%d".formatted(detectedID);
                        arUcoMarkerNode = new ArUcoMarkerNode(sceneGraph.getNextID().getAndIncrement(), nodeName, detectedID, markerSize);
                        LogTools.info("Adding detected ArUco marker {} to scene graph as {}", detectedID, nodeName);
                        modificationQueue.accept(new SceneGraphNodeAddition(arUcoMarkerNode, sceneGraph.getRootNode()));
                        sceneGraph.getArUcoMarkerIDToNodeMap().put(detectedID, arUcoMarkerNode); // Prevent it getting added twice
                     }
                  }
               }
            }

            DoorSceneNodeDefinitions.ensureNodesAdded(sceneGraph, modificationQueue);
            RigidBodySceneObjectDefinitions.ensureNodesAdded(sceneGraph, modificationQueue);
         });

         // All ArUco markers are child of root
         // This must be done after the above are added to the scene graph
         for (SceneNode child : sceneGraph.getRootNode().getChildren())
         {
            if (child instanceof ArUcoMarkerNode arUcoMarkerNode)
            {
               boolean isDetected = arUcoMarkerDetectionResults.isDetected(arUcoMarkerNode.getMarkerID());
               arUcoMarkerNode.setCurrentlyDetected(isDetected);
               if (isDetected)
               {
                  arUcoMarkerDetectionResults.getPose(arUcoMarkerNode.getMarkerID(),
                                               arUcoMarkerNode.getMarkerSize(),
                                               sensorFrame,
                                               arUcoMarkerNode.getNodeFrame().getParent(),
                                               arUcoMarkerNode.getNodeToParentFrameTransform());
                  arUcoMarkerNode.applyFilter();
                  arUcoMarkerNode.getNodeFrame().update();
               }
            }
         }
      }
   }
}
