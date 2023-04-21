package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.arUco.ArUcoMarkerInfo;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 */
public class ArUcoDetectableNode extends PredefinedRigidBodySceneNode
{
   private final int markerID;
   private final double markerSize;
   private final RigidBodyTransform markerTransformToParent = new RigidBodyTransform();

   /**
    * Give the marker info directly from code.
    */
   public ArUcoDetectableNode(String name,
                              int markerID,
                              double markerSize,
                              RigidBodyTransform markerTransformToParent,
                              String visualModelFilePath,
                              RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);

      this.markerID = markerID;
      this.markerSize = markerSize;
      this.markerTransformToParent.set(markerTransformToParent);
   }

   /**
    * Loads info from StoredPropertySet with name as suffix
    */
   public ArUcoDetectableNode(String name, String visualModelFilePath, RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);

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
