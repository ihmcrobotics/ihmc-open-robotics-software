package us.ihmc.perception.scene;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.arUco.ArUcoMarkerInfo;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 */
public class ArUcoDetectableObject extends KnownRigidModelSceneObject
{
   private final int markerID;
   private final double markerSize;
   private final RigidBodyTransform markerTransformToParent = new RigidBodyTransform();

   /**
    * Give the marker info directly from code.
    */
   public ArUcoDetectableObject(String name, int markerID, double markerSize, RigidBodyTransform markerTransformToParent, String visualModelFilePath)
   {
      super(name, visualModelFilePath);

      this.markerID = markerID;
      this.markerSize = markerSize;
      this.markerTransformToParent.set(markerTransformToParent);
   }

   /**
    * Loads info from StoredPropertySet with name as suffix
    */
   public ArUcoDetectableObject(String name, String visualModelFilePath)
   {
      super(name, visualModelFilePath);

      ArUcoMarkerInfo arUcoMarkerInfo = new ArUcoMarkerInfo(name);
      markerID = arUcoMarkerInfo.getMarkerID();
      markerSize = arUcoMarkerInfo.getMarkerSize();
      markerTransformToParent.getTranslation().set(arUcoMarkerInfo.getMarkerXTranslationToParent(),
                                                   arUcoMarkerInfo.getMarkerYTranslationToParent(),
                                                   arUcoMarkerInfo.getMarkerZTranslationToParent());
      EuclidCoreMissingTools.setYawPitchRollDegrees(markerTransformToParent.getRotation(),
                                                    arUcoMarkerInfo.getMarkerYawRotationToParentDegrees(),
                                                    arUcoMarkerInfo.getMarkerPitchRotationToParentDegrees(),
                                                    arUcoMarkerInfo.getMarkerRollRotationToParentDegrees());
   }

   public int getMarkerID()
   {
      return markerID;
   }

   public double getMarkerSize()
   {
      return markerSize;
   }

   public RigidBodyTransform getMarkerTransformToParent()
   {
      return markerTransformToParent;
   }
}
