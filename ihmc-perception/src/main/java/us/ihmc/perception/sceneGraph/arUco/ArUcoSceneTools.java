package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticArUcoRelativeDetectableSceneObject;
import us.ihmc.perception.sceneGraph.PredefinedSceneObjectLibrary;

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
         arUcoDetectableObject.setCurrentlyDetected(isDetected);
         if (isDetected)
         {
            arUcoMarkerDetection.getPose(arUcoDetectableObject.getMarkerID(),
                                         arUcoDetectableObject.getMarkerSize(),
                                         arUcoDetectableObject.getReferenceFrame().getParent(),
                                         arUcoDetectableObject.getTransformToParent());

            StaticArUcoRelativeDetectableSceneObject staticArUcoRelativeDetectableSceneObject
                  = predefinedSceneObjectLibrary.getStaticArUcoRelativeDetectableObjects().get(arUcoDetectableObject.getMarkerID());
            if (staticArUcoRelativeDetectableSceneObject != null)
            {
               Pose3DReadOnly poseInSensorFrame = arUcoMarkerDetection.getPoseInSensorFrame(arUcoDetectableObject.getMarkerID(),
                                                                                            arUcoDetectableObject.getMarkerSize());
               if (!staticArUcoRelativeDetectableSceneObject.getPoseKnown() &&
                   poseInSensorFrame.getPosition().norm() <= staticArUcoRelativeDetectableSceneObject.getMaximumDistanceToLockIn())
               {
                  arUcoMarkerDetection.getPose(staticArUcoRelativeDetectableSceneObject.getMarkerID(),
                                               staticArUcoRelativeDetectableSceneObject.getMarkerSize(),
                                               staticArUcoRelativeDetectableSceneObject.getReferenceFrame().getParent(),
                                               staticArUcoRelativeDetectableSceneObject.getTransformToParent());
                  staticArUcoRelativeDetectableSceneObject.lockInPose();
               }
            }
         }
      }
   }
}
