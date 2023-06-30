package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

public class ArUcoMarkerInfo
{
   private final String name;
   private final int markerID;
   private final double markerSize;
   private final ModifiableReferenceFrame markerFrame;

   /**
    *
    * @param markerToNodeFrameTransform we measure the marker like it's a child of the node
    *                                   but really it's the parent, so we'll invert it in here
    */
   public ArUcoMarkerInfo(String name, int markerID, double markerSize, RigidBodyTransform markerToNodeFrameTransform)
   {
      this.name = name;
      this.markerID = markerID;
      this.markerSize = markerSize;

      // We always represent ArUco markers in world frame
      markerFrame = new ModifiableReferenceFrame(name + "MarkerFrame", ReferenceFrame.getWorldFrame());
      getNodeToParentFrameTransform().setAndInvert(markerToNodeFrameTransform);
   }

   public int getMarkerID()
   {
      return markerID;
   }

   public double getMarkerSize()
   {
      return markerSize;
   }

   public ReferenceFrame getMarkerFrame()
   {
      return markerFrame.getReferenceFrame();
   }

   /**
    * Used to get and set the transform from marker frame to world frame.
    * If you modify this transform, you must then call {@link ReferenceFrame#update()} on {@link #getMarkerFrame()}.
    * @return the transform from marker frame to world frame
    */
   public RigidBodyTransform getMarkerToWorldFrameTransform()
   {
      return markerFrame.getTransformToParent();
   }
}
