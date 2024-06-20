package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;

public class CenterposeNode extends DetectableSceneNode
{
   private int objectID;
   private Point3D[] vertices3D;
   private Point3D[] vertices2D;
   private String objectType;
   private double confidence;
   private final RigidBodyTransform interpolatedModelTransform = new RigidBodyTransform();
   private int glitchCount;
   private boolean enableTracking;

   public CenterposeNode(long id, String name, int markerID, Point3D[] vertices3D, Point3D[] vertices2D, boolean enableTracking, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      this.objectID = markerID;
      this.vertices3D = vertices3D;
      this.vertices2D = vertices2D;
      this.enableTracking = enableTracking;
   }

   public void update()
   {
      RigidBodyTransform detectionTransform = getNodeToParentFrameTransform();
      FramePose3D detectionPose = new FramePose3D(ReferenceFrame.getWorldFrame(), detectionTransform);
      FramePose3D interpolatedModelTransformPose = new FramePose3D(ReferenceFrame.getWorldFrame(), interpolatedModelTransform);

      double distance = detectionPose.getPositionDistance(interpolatedModelTransformPose);

      boolean skipUpdate = false;
      if (distance > 0.5)
      {
         if (glitchCount < 5)
         {
            skipUpdate = true;
            glitchCount++;
         }
         else
         {
            glitchCount = 0;
         }
      }

      if (!skipUpdate && enableTracking)
      {
         double alpha = normalize(distance, 0.001, 1.0);
         alpha = MathTools.clamp(alpha, 0.001, 1);

         interpolatedModelTransform.interpolate(detectionTransform, alpha);
         getNodeToParentFrameTransform().set(interpolatedModelTransform);
         getNodeFrame().update();
      }
   }

   public static double normalize(double value, double min, double max) {
      return (value - min) / (max - min);
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

   public boolean isEnableTracking()
   {
      return enableTracking;
   }

   public void setEnableTracking(boolean enableTracking)
   {
      this.enableTracking = enableTracking;
   }
}
