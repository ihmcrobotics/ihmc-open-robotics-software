package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

import java.util.ArrayList;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 *
 * This node has a ReferenceFrame for the marker and the node's transform is a child of it.
 */
public class ArUcoDetectableNode extends PredefinedRigidBodySceneNode
{
   /**
    * It's possible to have more than one marker able to detect the pose of this object.
    * For instance, for a door, you want a marker on the "push" side and another one with
    * a different ID on the "pull" side, which give the pose of the same object: the door
    * panel.
    */
   private final ArrayList<ArUcoMarkerInfo> detectableMarkers = new ArrayList<>();

   /**
    * Give the marker info directly from code.
    */
   public ArUcoDetectableNode(String name, String visualModelFilePath, RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);

      changeParentFrame(markerFrame.getReferenceFrame());

      getNodeToParentFrameTransform().setAndInvert(markerToNodeFrameTransform);
      getNodeFrame().update();
   }

   public ArrayList<ArUcoMarkerInfo> getDetectableMarkers()
   {
      return detectableMarkers;
   }
}
