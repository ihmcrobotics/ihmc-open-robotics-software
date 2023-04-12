package us.ihmc.perception.scene;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.arUco.ArUcoMarkerInfo;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 */
public class ROS2ArUcoDetectableObject extends ROS2DetectableSceneObject
{
   private final long markerID;
   private final double markerSize;
   private final RigidBodyTransform markerTransformToParent = new RigidBodyTransform();

   /**
    * Give the marker info directly from code.
    */
   public ROS2ArUcoDetectableObject(String name, long markerID, double markerSize, RigidBodyTransform markerTransformToParent)
   {
      super(name);

      this.markerID = markerID;
      this.markerSize = markerSize;
      this.markerTransformToParent.set(markerTransformToParent);
   }

   /**
    * Loads info from StoredPropertySet with name as suffix
    */
   public ROS2ArUcoDetectableObject(String name)
   {
      super(name);

      ArUcoMarkerInfo arUcoMarkerInfo = new ArUcoMarkerInfo(name);
      markerID = (long) arUcoMarkerInfo.getMarkerID();
      markerSize = arUcoMarkerInfo.getMarkerSize();
      markerTransformToParent.getTranslation().set(arUcoMarkerInfo.getMarkerXTranslationToParent(),
                                                   arUcoMarkerInfo.getMarkerYTranslationToParent(),
                                                   arUcoMarkerInfo.getMarkerZTranslationToParent());
      EuclidCoreMissingTools.setYawPitchRollDegrees(markerTransformToParent.getRotation(),
                                                    arUcoMarkerInfo.getMarkerYawRotationToParentDegrees(),
                                                    arUcoMarkerInfo.getMarkerPitchRotationToParentDegrees(),
                                                    arUcoMarkerInfo.getMarkerRollRotationToParentDegrees());
   }
}
