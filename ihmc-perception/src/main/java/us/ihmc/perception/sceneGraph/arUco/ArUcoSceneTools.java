package us.ihmc.perception.sceneGraph.arUco;

import gnu.trove.iterator.TIntIterator;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;

/**
 * This class exists to perform some operations that are like "glue" between the scene based
 * objects and the OpenCV based ArUco marker detection.
 */
public class ArUcoSceneTools
{
   public static void updateSceneGraph(OpenCVArUcoMarkerDetection arUcoMarkerDetection, SceneGraph sceneGraph)
   {
      boolean modifiedSceneGraph = false;

      synchronized (arUcoMarkerDetection.getSyncObject())
      {
         for (TIntIterator iterator = arUcoMarkerDetection.getDetectedIDs().iterator(); iterator.hasNext(); )
         {
            int detectedID = iterator.next();
            ArUcoMarkerNode arUcoMarkerNode = sceneGraph.getArUcoMarkerIDToNodeMap().get(detectedID);
            if (arUcoMarkerNode == null) // Add ArUco marker node if it is missing
            {
               DetectionFilter candidateFilter = sceneGraph.getCandidateFiltration().getOrCreateFilter(detectedID);
               candidateFilter.registerDetection();
               if (candidateFilter.isStableDetectionResult())
               {
                  sceneGraph.getCandidateFiltration().removeFilter(detectedID);

                  String nodeName = "ArUcoMarker%d".formatted(detectedID);
                  arUcoMarkerNode = new ArUcoMarkerNode(sceneGraph.getNextID().getAndIncrement(),
                                                        nodeName,
                                                        detectedID,
                                                        RigidBodySceneObjectDefinitions.LARGE_MARKER_WIDTH);
                  LogTools.info("Adding detected ArUco marker {} to scene graph as {}", detectedID, nodeName);
                  sceneGraph.getRootNode().getChildren().add(arUcoMarkerNode);
                  sceneGraph.getRootNode().markModifiedByOperator();
                  sceneGraph.getArUcoMarkerIDToNodeMap().put(detectedID, arUcoMarkerNode);
                  modifiedSceneGraph = true;
               }
            }
         }

         // All ArUco markers are child of root
         for (SceneNode child : sceneGraph.getRootNode().getChildren())
         {
            if (child instanceof ArUcoMarkerNode arUcoMarkerNode)
            {
               boolean isDetected = arUcoMarkerDetection.isDetected(arUcoMarkerNode.getMarkerID());
               arUcoMarkerNode.setCurrentlyDetected(isDetected);
               if (isDetected)
               {
                  arUcoMarkerDetection.getPose(arUcoMarkerNode.getMarkerID(),
                                               arUcoMarkerNode.getMarkerSize(),
                                               arUcoMarkerNode.getNodeFrame().getParent(),
                                               arUcoMarkerNode.getNodeToParentFrameTransform());
                  arUcoMarkerNode.applyFilter();
                  arUcoMarkerNode.getNodeFrame().update();
               }
            }
         }
      }

      modifiedSceneGraph |= DoorSceneNodeDefinitions.ensureNodesAdded(sceneGraph);
      modifiedSceneGraph |= RigidBodySceneObjectDefinitions.ensureNodesAdded(sceneGraph);

      if (modifiedSceneGraph)
      {
         sceneGraph.updateCaches();
         sceneGraph.ensureFramesMatchParentsRecursively();
      }
   }
}
