package us.ihmc.perception.scene;

import us.ihmc.perception.OpenCVArUcoMarkerDetection;

/**
 * This class exists to perform some operations that are like "glue" between the scene based
 * objects and the OpenCV based ArUco marker detection.
 */
public class ArUcoSceneTools
{
   public static void updateLibraryPosesFromDetectionResults(OpenCVArUcoMarkerDetection arUcoMarkerDetection,
                                                             PredefinedSceneObjectLibrary predefinedSceneObjectLibrary)
   {
      for (ArUcoDetectableObject arUcoDetectableObject : predefinedSceneObjectLibrary.getArUcoDetectableObjects())
      {
         boolean isDetected = arUcoMarkerDetection.isDetected(arUcoDetectableObject.getMarkerID());
         if (isDetected)
         {
            arUcoMarkerDetection.getPose(arUcoDetectableObject.getMarkerID(),
                                         arUcoDetectableObject.getMarkerSize(),
                                         arUcoDetectableObject.getReferenceFrame().getParent(),
                                         arUcoDetectableObject.getTransformToParent());
         }
      }
   }
}
