package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;

public class CenterposeNode extends DetectableSceneNode
{
   private final PersistentDetection objectDetection;

   // PersistentDetection values stored locally for syncing purposes
   private final String objectType;
   private final int objectID;
   private double confidence;
   private Point3D[] boundingBoxVertices;
   private Point2D[] boundingBoxVertices2D;

   // CenterPoseNode specific variables
   private final RigidBodyTransform interpolatedModelTransform = new RigidBodyTransform();
   private int glitchCount;
   private boolean enableTracking;

   public CenterposeNode(long nodeID, String name, PersistentDetection objectDetection, boolean enableTracking, CRDTInfo crdtInfo)
   {
      super(nodeID, name, crdtInfo);
      this.enableTracking = enableTracking;
      this.objectDetection = objectDetection;

      objectType = objectDetection.getDetectedObjectName();
      objectID = Integer.parseInt(objectDetection.getDetectedObjectClass());
      confidence = getMostRecentDetection().getConfidence();
      boundingBoxVertices = getMostRecentDetection().getBoundingBoxVertices();
      boundingBoxVertices2D = getMostRecentDetection().getBoundingBoxVertices2D();
   }

   /**
    * Constructor used when the node does not have access to the persistent detection (e.g. UI side).
    * All values that would typically come from the persistent detection must be synced separately.
    */
   public CenterposeNode(long nodeID,
                         String name,
                         boolean enableTracking,
                         String objectType,
                         int objectID,
                         double confidence,
                         Point3D[] boundingBoxVertices,
                         Point2D[] boundingBoxVertices2D,
                         CRDTInfo crdtInfo)
   {
      super(nodeID, name, crdtInfo);

      this.enableTracking = enableTracking;
      this.objectType = objectType;
      this.objectID = objectID;
      this.confidence = confidence;
      this.boundingBoxVertices = boundingBoxVertices;
      this.boundingBoxVertices2D = boundingBoxVertices2D;
      this.objectDetection = null;
   }

   @Override
   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      super.update(sceneGraph, modificationQueue);

      setCurrentlyDetected(objectDetection.isStable());
      getNodeToParentFrameTransform().set(getMostRecentDetection().getPose());

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

      confidence = getMostRecentDetection().getConfidence();
      boundingBoxVertices = getMostRecentDetection().getBoundingBoxVertices();
      boundingBoxVertices2D = getMostRecentDetection().getBoundingBoxVertices2D();
   }

   public static double normalize(double value, double min, double max)
   {
      return (value - min) / (max - min);
   }

   public Point3D[] getVertices3D()
   {
      return boundingBoxVertices;
   }

   public void setBoundingBoxVertices(Point3D[] boundingBoxVertices)
   {
      this.boundingBoxVertices = boundingBoxVertices;
   }

   public Point2D[] getVertices2D()
   {
      return boundingBoxVertices2D;
   }

   public void setBoundingBoxVertices2D(Point2D[] boundingBoxVertices2D)
   {
      this.boundingBoxVertices2D = boundingBoxVertices2D;
   }

   public String getObjectType()
   {
      return objectType;
   }

   public int getObjectID()
   {
      return objectID;
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

   public PersistentDetection getCenterPoseDetection()
   {
      return objectDetection;
   }

   public CenterPoseInstantDetection getMostRecentDetection()
   {
      return (CenterPoseInstantDetection) objectDetection.getMostRecentDetection();
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);
      if (objectDetection != null)
         objectDetection.markForDeletion();
   }
}
