package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.euclid.tuple3D.Point3D;
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
   private int markerID;
   private long sequenceID;
   private double breakFrequency = 1.0;
   private Point3D[] vertices3D = new Point3D[8];
   private Point3D[] vertices2D = new Point3D[8];
   private String object_type;

   public CenterposeNode(long id, String name, int markerID, Point3D[] vertices3D, Point3D[] vertices2D)
   {
      super(id, name);
      this.markerID = markerID;
      this.vertices3D = vertices3D;
      this.vertices2D = vertices2D;
   }

   public long getSequenceID()
   {
      return sequenceID;
   }

   public void setSequenceID(long sequenceID)
   {
      this.sequenceID = sequenceID;
   }

   public Point3D[] getVertices2D()
   {
      return vertices2D;
   }

   public void setVertices2D(Point3D[] vertices2D)
   {
      this.vertices2D = vertices2D;
   }

   public double getConfidence()
   {
      return confidence;
   }

   public void setConfidence(double confidence)
   {
      this.confidence = confidence;
   }

   private double confidence;

   public String getObject_type()
   {
      return object_type;
   }

   public void setObject_type(String object_type)
   {
      this.object_type = object_type;
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

   public void setMarkerID(int markerID)
   {
      this.markerID = markerID;
   }

   public double getBreakFrequency()
   {
      return breakFrequency;
   }

   public void setBreakFrequency(double breakFrequency)
   {
      this.breakFrequency = breakFrequency;
   }

   public void setVertices3D(Point3D[] vertices3D)
   {
      this.vertices3D = vertices3D;
   }

   public Point3D[] getVertices3D()
   {
      return vertices3D;
   }
}
