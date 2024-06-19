package us.ihmc.perception.sceneGraph.centerpose;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;

public class CenterposeNode extends DetectableSceneNode
{
   private CenterPoseInstantDetection centerPoseDetection;

   private final RigidBodyTransform interpolatedModelTransform = new RigidBodyTransform();
   private int glitchCount;
   private boolean enableTracking;

   public CenterposeNode(long nodeID, String name, CenterPoseInstantDetection centerPoseDetection, boolean enableTracking)
   {
      super(nodeID, name, centerPoseDetection);
      this.centerPoseDetection = centerPoseDetection;
      this.enableTracking = enableTracking;
   }

   @Override
   public void updateDetection(InstantDetection newDetection)
   {
      if (newDetection instanceof CenterPoseInstantDetection newCenterPoseDetection)
      {
         super.updateDetection(newCenterPoseDetection);
         this.centerPoseDetection = newCenterPoseDetection;

         getNodeToParentFrameTransform().set(centerPoseDetection.getPose());

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
      else
         throw new IllegalArgumentException("CenterPoseNode update requires a CenterPoseInstantDetection");
   }

   public static double normalize(double value, double min, double max) {
      return (value - min) / (max - min);
   }

   public Point3D[] getVertices3D()
   {
      return centerPoseDetection.getBoundingBoxVertices();
   }

   public Point2D[] getVertices2D()
   {
      return centerPoseDetection.getBoundingBoxVertices2D();
   }

   public String getObjectType()
   {
      return centerPoseDetection.getDetectedObjectName();
   }

   public double getConfidence()
   {
      return centerPoseDetection.getConfidence();
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
