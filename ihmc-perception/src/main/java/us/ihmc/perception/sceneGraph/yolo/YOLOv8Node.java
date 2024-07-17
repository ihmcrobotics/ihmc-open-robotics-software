package us.ihmc.perception.sceneGraph.yolo;

import perception_msgs.msg.dds.YOLOv8NodeMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;

import java.util.ArrayList;
import java.util.List;

public class YOLOv8Node extends DetectableSceneNode
{
   private final PersistentDetection yoloDetection;

   // PersistentDetection values stored locally for syncing purposes
   private double confidence = 0.0;
   private List<Point3D32> objectPointCloud = new ArrayList<>();

   // YOLOv8Node specific variables
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private final Pose3D objectPose;

   /**
    * Constructor used when the node does not have access to the persistent detection (e.g. UI side).
    * All values that would typically come from the persistent detection must be synced separately.
    *
    * NOTE: the object point cloud is not deep copied. This means any changes to the data contained in the point cloud will affect
    * this object.
    */
   public YOLOv8Node(long id,
                     String name,
                     double confidence,
                     List<Point3D32> objectPointCloud,
                     RigidBodyTransformReadOnly centroidToObjectTransform,
                     Pose3DReadOnly objectPose,
                     CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);

      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = new Pose3D(objectPose);
      this.confidence = confidence;
      this.objectPointCloud = objectPointCloud;
      yoloDetection = null;
   }

   // TODO: remove?
   public YOLOv8Node(long id, String name, CRDTInfo crdtInfo, PersistentDetection detection)
   {
      super (id, name, crdtInfo);

      this.yoloDetection = detection;
      objectPose = new Pose3D();
   }

   @Override
   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      super.update(sceneGraph, modificationQueue);

      YOLOv8InstantDetection mostRecentDetection = (YOLOv8InstantDetection) yoloDetection.getMostRecentDetection();
      setCurrentlyDetected(yoloDetection.isStable());
      setConfidence(mostRecentDetection.getConfidence());
      setObjectPointCloud(mostRecentDetection.getObjectPointCloud());

      objectPose.set(mostRecentDetection.getPose());
      objectPose.appendTransform(centroidToObjectTransform);

      setNodeToParentFrameTransformAndUpdate(objectPose);
   }

   public RigidBodyTransformReadOnly getCentroidToObjectTransform()
   {
      return centroidToObjectTransform;
   }

   public double getConfidence()
   {
      return confidence;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public Point3DReadOnly getObjectPoint(int index)
   {
      return objectPointCloud.get(index);
   }

   public Pose3DReadOnly getObjectPose()
   {
      return objectPose;
   }

   public void toMessage(YOLOv8NodeMessage message)
   {
      message.setConfidence(getConfidence());
      message.getObjectPointCloud().clear();
      for (int i = 0; i < message.getObjectPointCloud().getCurrentCapacity() && i < getObjectPointCloud().size(); ++i)
      {
         message.getObjectPointCloud().add().set(getObjectPoint(i));
      }
      message.getCentroidToObjectTransform().set(getCentroidToObjectTransform());
      message.getObjectPose().set(getObjectPose());
      message.getFilteredObjectPose().set(getObjectPose()); // FIXME Maybe set this to something else?
   }

   public void fromMessage(YOLOv8NodeMessage message)
   {
      setConfidence(message.getConfidence());
      setObjectPointCloud(message.getObjectPointCloud());
      setCentroidToObjectTransform(message.getCentroidToObjectTransform());
      setObjectPose(message.getObjectPose());
   }

   public void setConfidence(double confidence)
   {
      this.confidence = confidence;
   }

   public void setObjectPointCloud(List<Point3D32> objectPointCloud)
   {
      this.objectPointCloud = objectPointCloud;
   }

   public void setObjectPose(Pose3DReadOnly objectPose)
   {
      this.objectPose.set(objectPose);
   }

   public void setCentroidToObjectTransform(RigidBodyTransformReadOnly centroidToObjectTransform)
   {
      this.centroidToObjectTransform.set(centroidToObjectTransform);
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);
      if (yoloDetection != null)
         yoloDetection.markForDeletion();
   }
}
