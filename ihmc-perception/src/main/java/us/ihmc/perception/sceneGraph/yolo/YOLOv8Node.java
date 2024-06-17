package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;

import java.util.List;

public class YOLOv8Node extends DetectableSceneNode
{
   private YOLOv8InstantDetection yoloDetection;
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private Pose3D objectPose;

   public YOLOv8Node(long id, String name, YOLOv8InstantDetection detection)
   {
      this(id,
           name,
           detection,
           new Pose3D(),
           detection.getPose());
   }

   public YOLOv8Node(long id,
                     String name,
                     YOLOv8InstantDetection detection,
                     RigidBodyTransformBasics centroidToObjectTransform,
                     Pose3D objectPose)
   {
      super(id, name, detection);

      this.yoloDetection = detection;
      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = objectPose;
   }

   @Override
   public void updateDetection(InstantDetection newDetection)
   {
      if (newDetection instanceof YOLOv8InstantDetection newYoloDetection)
      {
         super.updateDetection(newYoloDetection);
         yoloDetection = newYoloDetection;

         objectPose.set(getDetection().getPose());
         objectPose.appendTransform(centroidToObjectTransform);

         getNodeToParentFrameTransform().set(objectPose);
         getNodeFrame().update();
      }
      else
         throw new IllegalArgumentException("YOLOv8Node update requires a YOLOv8InstantDetection");
   }

   public RigidBodyTransform getCentroidToObjectTransform()
   {
      return centroidToObjectTransform;
   }

   public void setCentroidToObjectTransform(RigidBodyTransformBasics centroidToObjectTransform)
   {
      this.centroidToObjectTransform.set(centroidToObjectTransform);
   }

   public YOLOv8InstantDetection getYoloDetection()
   {
      return yoloDetection;
   }

   public YOLOv8DetectionClass getDetectionClass()
   {
      return YOLOv8DetectionClass.fromName(yoloDetection.getDetectedObjectName());
   }

   public Pose3D getObjectPose()
   {
      return objectPose;
   }

   public void setObjectPose(Pose3D objectPose)
   {
      this.objectPose = objectPose;
   }
}
