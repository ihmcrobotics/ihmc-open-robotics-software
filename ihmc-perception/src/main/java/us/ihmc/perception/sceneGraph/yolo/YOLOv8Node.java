package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public class YOLOv8Node extends DetectableSceneNode
{
   private final PersistentDetection yoloDetection;

   // PersistentDetection values stored locally for syncing purposes
   private double confidence = 0.0;
   private List<Point3D32> objectPointCloud = new ArrayList<>();

   // YOLOv8Node specific variables
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private Pose3D objectPose;

   public YOLOv8Node(long id, String name, PersistentDetection yoloDetection, CRDTInfo crdtInfo)
   {
      this(id,
           name,
           yoloDetection,
           new Pose3D(),
           yoloDetection.getMostRecentDetection().getPose(),
           crdtInfo);
   }

   public YOLOv8Node(long id,
                     String name,
                     PersistentDetection yoloDetection,
                     RigidBodyTransformBasics centroidToObjectTransform,
                     Pose3D objectPose,
                     CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);

      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = objectPose;
      this.yoloDetection = yoloDetection;
      confidence = getMostRecentDetection().getConfidence();
      objectPointCloud = getMostRecentDetection().getObjectPointCloud();
   }

   /**
    * Constructor used when the node does not have access to the persistent detection (e.g. UI side).
    * All values that would typically come from the persistent detection must be synced separately.
    */
   public YOLOv8Node(long id,
                     String name,
                     double confidence,
                     List<Point3D32> objectPointCloud,
                     RigidBodyTransformBasics centroidToObjectTransform,
                     Pose3D objectPose,
                     CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);

      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = objectPose;
      this.confidence = confidence;
      this.objectPointCloud = objectPointCloud;
      yoloDetection = null;
   }

   public void update()
   {
      setCurrentlyDetected(yoloDetection.isStable());
      setConfidence(getMostRecentDetection().getConfidence());
      setObjectPointCloud(getMostRecentDetection().getObjectPointCloud());

      objectPose.set(yoloDetection.getMostRecentDetection().getPose());
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

   public PersistentDetection getYOLODetection()
   {
      return yoloDetection;
   }

   public YOLOv8InstantDetection getMostRecentDetection()
   {
      return (YOLOv8InstantDetection) yoloDetection.getMostRecentDetection();
   }

   public double getConfidence()
   {
      return confidence;
   }

   public void setConfidence(double confidence)
   {
      this.confidence = confidence;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public void setObjectPointCloud(List<Point3D32> objectPointCloud)
   {
      this.objectPointCloud = objectPointCloud;
   }

   public Pose3D getObjectPose()
   {
      return objectPose;
   }

   public void setObjectPose(Pose3D objectPose)
   {
      this.objectPose = objectPose;
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);
      if (yoloDetection != null)
         yoloDetection.markForDeletion();
   }
}
