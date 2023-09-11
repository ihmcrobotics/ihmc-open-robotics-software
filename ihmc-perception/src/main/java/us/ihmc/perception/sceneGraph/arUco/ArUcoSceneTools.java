package us.ihmc.perception.sceneGraph.arUco;

import gnu.trove.iterator.TIntIterator;
import gnu.trove.set.TIntSet;
import gnu.trove.set.hash.TIntHashSet;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorModelParameters;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;

/**
 * This class exists to perform some operations that are like "glue" between the scene based
 * objects and the OpenCV based ArUco marker detection.
 */
public class ArUcoSceneTools
{
   public static void updateLibraryPosesFromDetectionResults(OpenCVArUcoMarkerDetection arUcoMarkerDetection, SceneGraph sceneGraph)
   {
      synchronized (arUcoMarkerDetection.getSyncObject())
      {
         // Add objects to scene if they are missing
         TIntSet missingIDSet = new TIntHashSet(arUcoMarkerDetection.getDetectedIDs());

         // ArUco markers are all at the first level
         for (SceneNode child : sceneGraph.getRootNode().getChildren())
         {
            if (child instanceof ArUcoMarkerNode arUcoMarkerNode)
            {
               missingIDSet.remove(arUcoMarkerNode.getMarkerID());
            }
         }

         for (TIntIterator iterator = missingIDSet.iterator(); iterator.hasNext(); )
         {
            int missingMarkerID = iterator.next();
            LogTools.info("Adding detected ArUco marker to scene graph: {}", missingMarkerID);
            ArUcoMarkerNode arUcoMarkerNode = new ArUcoMarkerNode(sceneGraph.getNextID().getAndIncrement(),
                                                                  "ArUcoMarker%d".formatted(missingMarkerID),
                                                                  missingMarkerID,
                                                                  RigidBodySceneObjectDefinitions.LARGE_MARKER_WIDTH);
            sceneGraph.getRootNode().getChildren().add(arUcoMarkerNode);
         }

         for (SceneNode child : sceneGraph.getRootNode().getChildren())
         {
            if (child instanceof ArUcoMarkerNode arUcoMarkerNode)
            {
               addPredfinedChildrenIfMissing(sceneGraph, arUcoMarkerNode);

               boolean isDetected = arUcoMarkerDetection.isDetected(arUcoMarkerNode.getMarkerID());
               arUcoMarkerNode.setCurrentlyDetected(isDetected);
               if (isDetected)
               {
                  arUcoMarkerDetection.getPose(arUcoMarkerNode.getMarkerID(),
                                               arUcoMarkerNode.getMarkerSize(),
                                               arUcoMarkerNode.getNodeFrame().getParent(), arUcoMarkerNode.getNodeToParentFrameTransform());
                  arUcoMarkerNode.applyFilter();
                  arUcoMarkerNode.getNodeFrame().update();
               }
            }
         }
      }
   }

   public static void addPredfinedChildrenIfMissing(SceneGraph sceneGraph, ArUcoMarkerNode arUcoMarkerNode)
   {
      switch (arUcoMarkerNode.getMarkerID())
      {
         case DoorModelParameters.PULL_DOOR_MARKER_ID:
         {
            DoorSceneNodeDefinitions.createPullDoorLeverHandle(sceneGraph, arUcoMarkerNode);
            PredefinedRigidBodySceneNode pullDoorPanel = DoorSceneNodeDefinitions.createPullDoorPanel(sceneGraph, arUcoMarkerNode);
            DoorSceneNodeDefinitions.createPullDoorFrame(sceneGraph, pullDoorPanel);
         }
         case DoorModelParameters.PUSH_DOOR_MARKER_ID:
         {
            DoorSceneNodeDefinitions.createPushDoorLeverHandle(sceneGraph, arUcoMarkerNode);
            PredefinedRigidBodySceneNode pushDoorPanel = DoorSceneNodeDefinitions.createPushDoorPanel(sceneGraph, arUcoMarkerNode);
            DoorSceneNodeDefinitions.createPushDoorFrame(sceneGraph, pushDoorPanel);
         }
         case RigidBodySceneObjectDefinitions.BOX_MARKER_ID:
         {
            RigidBodySceneObjectDefinitions.createBox(sceneGraph, arUcoMarkerNode);
         }
         case RigidBodySceneObjectDefinitions.CAN_OF_SOUP_MARKER_ID:
         {
            RigidBodySceneObjectDefinitions.createCanOfSoup(sceneGraph, arUcoMarkerNode);
         }
      }

      arUcoMarkerNode.update();
   }
}
