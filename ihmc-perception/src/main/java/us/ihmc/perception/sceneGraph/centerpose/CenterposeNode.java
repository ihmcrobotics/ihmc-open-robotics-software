package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.perception.filters.BreakFrequencyAlphaCalculator;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.robotics.math.filters.AlphaFilteredRigidBodyTransform;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 * <p>
 * This node has a ReferenceFrame for the marker and the node's transform is a child of it.
 */
public class CenterposeNode extends DetectableSceneNode
{
   private final AlphaFilteredRigidBodyTransform alphaFilteredTransformToParent = new AlphaFilteredRigidBodyTransform();
   private final BreakFrequencyAlphaCalculator breakFrequencyAlphaCalculator = new BreakFrequencyAlphaCalculator();
   private final int markerID;
   private double breakFrequency = 1.0;

   public CenterposeNode(long id, String name, int markerID)
   {
      super(id, name);
      this.markerID = markerID;
   }

   public void applyFilter()
   {
      alphaFilteredTransformToParent.setAlpha(breakFrequencyAlphaCalculator.calculateAlpha(breakFrequency));
      alphaFilteredTransformToParent.update(getNodeToParentFrameTransform());
      getNodeToParentFrameTransform().set(alphaFilteredTransformToParent);
   }

   public int getMarkerID()
   {
      return markerID;
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
