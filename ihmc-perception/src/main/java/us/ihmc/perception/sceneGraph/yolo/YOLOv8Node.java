package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;

import java.util.UUID;

public class YOLOv8Node extends DetectableSceneNode
{
   private final UUID detectionUUID;
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private Pose3D objectPose;

   public YOLOv8Node(long id, String name, YOLOv8InstantDetection detection, CRDTInfo crdtInfo)
   {
      this(id,
           name,
           detection,
           new Pose3D(),
           detection.getPose(),
           crdtInfo);
   }

   public YOLOv8Node(long id,
                     String name,
                     YOLOv8InstantDetection detection,
                     RigidBodyTransformBasics centroidToObjectTransform,
                     Pose3D objectPose,
                     CRDTInfo crdtInfo)
   {
      super(id, name, detection, crdtInfo);

      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = objectPose;
      detectionUUID = detection.getPersistentDetectionID();
   }

   @Override
   public void update(SceneGraph sceneGraph)
   {
      super.update(sceneGraph);

      objectPose.set(getDetection(detectionUUID).getPose());
      objectPose.appendTransform(centroidToObjectTransform);

      getNodeToParentFrameTransform().set(objectPose);
      getNodeFrame().update();
   }

   public RigidBodyTransform getCentroidToObjectTransform()
   {
      return centroidToObjectTransform;
   }

   public void setCentroidToObjectTransform(RigidBodyTransformBasics centroidToObjectTransform)
   {
      this.centroidToObjectTransform.set(centroidToObjectTransform);
   }

   public YOLOv8InstantDetection getYOLODetection()
   {
      return (YOLOv8InstantDetection) getDetection(detectionUUID);
   }

   public YOLOv8DetectionClass getDetectionClass()
   {
      return YOLOv8DetectionClass.fromName(getYOLODetection().getDetectedObjectName());
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
