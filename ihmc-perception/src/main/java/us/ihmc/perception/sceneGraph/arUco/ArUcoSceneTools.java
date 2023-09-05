package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
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
               arUcoDetectableNode.applyFilter();
               arUcoDetectableNode.getMarkerFrame().update();
            }
         }
      }
   }
}
