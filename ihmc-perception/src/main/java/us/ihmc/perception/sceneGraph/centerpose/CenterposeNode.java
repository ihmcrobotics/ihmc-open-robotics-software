package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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

   private final RigidBodyTransform previousTransform = new RigidBodyTransform();
   private final RigidBodyTransform updatedTransform = new RigidBodyTransform();

   private final FramePose3D lastLocalizedPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FramePose3D updatedPose = new FramePose3D(ReferenceFrame.getWorldFrame());

   private final AlphaFilteredRigidBodyTransform alphaFilteredTransformToParent = new AlphaFilteredRigidBodyTransform();
   private final BreakFrequencyAlphaCalculator breakFrequencyAlphaCalculator = new BreakFrequencyAlphaCalculator();

   private int glitchCount;

   public CenterposeNode(long id, String name, int markerID, Point3D[] vertices3D, Point3D[] vertices2D)
   {
      super(id, name);
      this.objectID = markerID;
      this.vertices3D = vertices3D;
      this.vertices2D = vertices2D;
   }

   public void update()
   {
      updatedTransform.set(getNodeToParentFrameTransform());
      updatedPose.set(updatedTransform);

      double distance = lastLocalizedPose.getPositionDistance(updatedPose);
      double threshold = 0.05;

      boolean skipUpdate = false;

      if (distance > threshold)
      {
         skipUpdate = true;
         glitchCount++;

         // While the detection is "glitching", set the confidence to low
         setConfidence(0.1);

         if (glitchCount > 5)
         {
            alphaFilteredTransformToParent.set(updatedTransform);
            lastLocalizedPose.set(updatedPose);

            glitchCount = 0;
            skipUpdate = false;
         }
      }
      else
      {
         // Filter over a localized zone
         double breakFrequency = (distance + 0.1) * 4;
         alphaFilteredTransformToParent.setAlpha(breakFrequencyAlphaCalculator.calculateAlpha(breakFrequency));
         alphaFilteredTransformToParent.update(updatedTransform);
         updatedTransform.set(alphaFilteredTransformToParent);
         lastLocalizedPose.set(updatedTransform);
      }

      if (!skipUpdate)
      {
         getNodeToParentFrameTransform().set(updatedTransform);
         getNodeFrame().update();
      }

      previousTransform.set(getNodeToParentFrameTransform());
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
}
