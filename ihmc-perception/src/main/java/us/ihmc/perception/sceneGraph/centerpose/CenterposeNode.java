package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.filters.BreakFrequencyAlphaCalculator;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.robotics.math.filters.AlphaFilteredRigidBodyTransform;

public class CenterposeNode extends DetectableSceneNode
{
   private int objectID;
   private Point3D[] vertices3D;
   private Point3D[] vertices2D;
   private String objectType;
   private double confidence;

   private final AlphaFilteredRigidBodyTransform alphaFilteredTransformToParent = new AlphaFilteredRigidBodyTransform();
   private final BreakFrequencyAlphaCalculator breakFrequencyAlphaCalculator = new BreakFrequencyAlphaCalculator();
   private double breakFrequency = 1.0;

   public CenterposeNode(long id, String name, int markerID, Point3D[] vertices3D, Point3D[] vertices2D)
   {
      super(id, name);
      this.objectID = markerID;
      this.vertices3D = vertices3D;
      this.vertices2D = vertices2D;
   }

   public void update()
   {
      alphaFilteredTransformToParent.setAlpha(breakFrequencyAlphaCalculator.calculateAlpha(breakFrequency));
      alphaFilteredTransformToParent.update(getNodeToParentFrameTransform());
      getNodeToParentFrameTransform().set(alphaFilteredTransformToParent);
      getNodeFrame().update();
   }

   public int getObjectID()
   {
      return objectID;
   }

   public void setObjectID(int objectID)
   {
      this.objectID = objectID;
   }

   public Point3D[] getVertices3D()
   {
      return vertices3D;
   }

   public void setVertices3D(Point3D[] vertices3D)
   {
      this.vertices3D = vertices3D;
   }

   public Point3D[] getVertices2D()
   {
      return vertices2D;
   }

   public void setVertices2D(Point3D[] vertices2D)
   {
      this.vertices2D = vertices2D;
   }

   public String getObjectType()
   {
      return objectType;
   }

   public void setObjectType(String objectType)
   {
      this.objectType = objectType;
   }

   public double getConfidence()
   {
      return confidence;
   }

   public void setConfidence(double confidence)
   {
      this.confidence = confidence;
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
