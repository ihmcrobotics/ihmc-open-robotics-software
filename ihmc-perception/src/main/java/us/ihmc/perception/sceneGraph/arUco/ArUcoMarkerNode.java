package us.ihmc.perception.sceneGraph.arUco;

import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.filters.BreakFrequencyAlphaCalculator;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.robotics.math.filters.AlphaFilteredRigidBodyTransform;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 *
 * This node has a ReferenceFrame for the marker and the node's transform is a child of it.
 */
public class ArUcoMarkerNode extends DetectableSceneNode
{
   private final AlphaFilteredRigidBodyTransform alphaFilteredTransformToParent = new AlphaFilteredRigidBodyTransform();
   private final BreakFrequencyAlphaCalculator breakFrequencyAlphaCalculator = new BreakFrequencyAlphaCalculator();
   private int markerID;
   private double markerSize;
   private double breakFrequency = 1.0;

   /**
    * Give the marker info directly from code.
    */
   public ArUcoMarkerNode(long id, String name, int markerID, double markerSize)
   {
      super(id, name, (PersistentDetection<? extends InstantDetection>) null); // TODO: FIXME TOMASZ PLEASE

      this.markerID = markerID;
      this.markerSize = markerSize;
   }

   public void applyFilter()
   {
      alphaFilteredTransformToParent.setAlpha(breakFrequencyAlphaCalculator.calculateAlpha(breakFrequency));
      alphaFilteredTransformToParent.update(getNodeToParentFrameTransform());
      getNodeToParentFrameTransform().set(alphaFilteredTransformToParent);
   }

   public void setMarkerID(int markerID)
   {
      this.markerID = markerID;
   }

   public int getMarkerID()
   {
      return markerID;
   }

   public void setMarkerSize(double markerSize)
   {
      this.markerSize = markerSize;
   }

   public double getMarkerSize()
   {
      return markerSize;
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
