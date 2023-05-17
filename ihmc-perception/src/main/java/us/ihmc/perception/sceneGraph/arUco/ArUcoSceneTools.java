package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticArUcoRelativeDetectableSceneNode;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;

/**
 * This class exists to perform some operations that are like "glue" between the scene based
 * objects and the OpenCV based ArUco marker detection.
 */
public class ArUcoSceneTools
{
   public static void updateLibraryPosesFromDetectionResults(OpenCVArUcoMarkerDetection arUcoMarkerDetection,
                                                             PredefinedSceneNodeLibrary predefinedSceneNodeLibrary)
   {
      synchronized (arUcoMarkerDetection.getSyncObject())
      {
         for (ArUcoDetectableNode arUcoDetectableNode : predefinedSceneNodeLibrary.getArUcoDetectableNodes())
         {
            boolean isDetected = arUcoMarkerDetection.isDetected(arUcoDetectableNode.getMarkerID());
            arUcoDetectableNode.setCurrentlyDetected(isDetected);
            if (isDetected)
            {
               arUcoMarkerDetection.getPose(arUcoDetectableNode.getMarkerID(),
                                            arUcoDetectableNode.getMarkerSize(),
                                            arUcoDetectableNode.getMarkerFrame().getParent(),
                                            arUcoDetectableNode.getMarkerToWorldFrameTransform());
               arUcoDetectableNode.getMarkerFrame().update();
               arUcoDetectableNode.getNodeToParentFrameTransform().setAndInvert(arUcoDetectableNode.getMarkerToNodeFrameTransform());
               arUcoDetectableNode.getNodeFrame().update();

               StaticArUcoRelativeDetectableSceneNode staticArUcoRelativeDetectableSceneNode = predefinedSceneNodeLibrary.getStaticArUcoRelativeDetectableNodes()
                                                                                                                         .get(arUcoDetectableNode.getMarkerID());
               if (staticArUcoRelativeDetectableSceneNode != null)
               {
                  Pose3DReadOnly poseInSensorFrame = arUcoMarkerDetection.getPoseInSensorFrame(arUcoDetectableNode.getMarkerID(), arUcoDetectableNode.getMarkerSize());
                  if (!staticArUcoRelativeDetectableSceneNode.getPoseKnown()
                      && poseInSensorFrame.getPosition().norm() <= staticArUcoRelativeDetectableSceneNode.getMaximumDistanceToLockIn())
                  {
                     arUcoMarkerDetection.getPose(staticArUcoRelativeDetectableSceneNode.getMarkerID(),
                                                  staticArUcoRelativeDetectableSceneNode.getMarkerSize(),
                                                  staticArUcoRelativeDetectableSceneNode.getMarkerFrame().getParent(),
                                                  staticArUcoRelativeDetectableSceneNode.getMarkerToWorldFrameTransform());
                     staticArUcoRelativeDetectableSceneNode.getMarkerFrame().update();
                     staticArUcoRelativeDetectableSceneNode.lockInPose();
                  }
               }
            }
         }
      }
   }
}
