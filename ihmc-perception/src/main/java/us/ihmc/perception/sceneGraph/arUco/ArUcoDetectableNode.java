package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.filters.BreakFrequencyAlphaCalculator;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.robotics.math.filters.AlphaFilteredRigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 *
 * This node has a ReferenceFrame for the marker and the node's transform is a child of it.
 */
public class ArUcoDetectableNode extends PredefinedRigidBodySceneNode
{
   private final int markerID;
   private final double markerSize;
   private final ModifiableReferenceFrame markerFrame;
   private final AlphaFilteredRigidBodyTransform alphaFilteredTransformToParent = new AlphaFilteredRigidBodyTransform();
   private final BreakFrequencyAlphaCalculator breakFrequencyAlphaCalculator = new BreakFrequencyAlphaCalculator();
   private double breakFrequency = 1.0;
   private transient final FramePose3D originalPose = new FramePose3D();

   /**
    * Give the marker info directly from code.
    *
    * @param markerToNodeFrameTransform we measure the marker like it's a child of the node
    *                                   but really it's the parent, so we'll invert it in here
    */
   public ArUcoDetectableNode(String name,
                              int markerID,
                              double markerSize,
                              RigidBodyTransform markerToNodeFrameTransform,
                              String visualModelFilePath,
                              RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);

      markerFrame = new ModifiableReferenceFrame(name + "MarkerFrame", ReferenceFrame.getWorldFrame());
      changeParentFrame(markerFrame.getReferenceFrame());
      setParentFrame(markerFrame::getReferenceFrame);

      this.markerID = markerID;
      this.markerSize = markerSize;
      getNodeToParentFrameTransform().setAndInvert(markerToNodeFrameTransform);
      getNodeFrame().update();

      setOriginalTransformToParent(getNodeToParentFrameTransform());
   }

   public void applyFilter()
   {
      alphaFilteredTransformToParent.setAlpha(breakFrequencyAlphaCalculator.calculateAlpha(breakFrequency));
      alphaFilteredTransformToParent.update(getMarkerToWorldFrameTransform());
      getMarkerToWorldFrameTransform().set(alphaFilteredTransformToParent);
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

   public double getBreakFrequency()
   {
      return breakFrequency;
   }

   public void setBreakFrequency(double breakFrequency)
   {
      this.breakFrequency = breakFrequency;
   }
}
