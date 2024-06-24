package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;

import java.util.UUID;

public class CenterposeNode extends DetectableSceneNode
{
   private final UUID detectionUUID;
   private final RigidBodyTransform interpolatedModelTransform = new RigidBodyTransform();
   private int glitchCount;
   private boolean enableTracking;

   public CenterposeNode(long nodeID, String name, CenterPoseInstantDetection centerPoseDetection, boolean enableTracking, CRDTInfo crdtInfo)
   {
      super(nodeID, name, centerPoseDetection, crdtInfo);
      this.enableTracking = enableTracking;
      detectionUUID = centerPoseDetection.getPersistentDetectionID();
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      super.update(sceneGraph);

      getNodeToParentFrameTransform().set(getDetection(detectionUUID).getPose());

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

   public Point3D[] getVertices3D()
   {
      return getCenterPoseDetection().getBoundingBoxVertices();
   }

   public Point2D[] getVertices2D()
   {
      return getCenterPoseDetection().getBoundingBoxVertices2D();
   }

   public String getObjectType()
   {
      return getCenterPoseDetection().getDetectedObjectName();
   }

   public double getConfidence()
   {
      return getCenterPoseDetection().getConfidence();
   }

   public boolean isEnableTracking()
   {
      return enableTracking;
   }

   public void setEnableTracking(boolean enableTracking)
   {
      this.enableTracking = enableTracking;
   }

   public CenterPoseInstantDetection getCenterPoseDetection()
   {
      return (CenterPoseInstantDetection) getDetection(detectionUUID);
   }
}
