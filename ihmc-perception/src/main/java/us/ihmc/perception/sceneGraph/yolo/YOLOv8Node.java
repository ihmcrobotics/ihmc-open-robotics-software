package us.ihmc.perception.sceneGraph.yolo;

import perception_msgs.msg.dds.YOLOv8NodeMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;

import java.time.Instant;

public class YOLOv8Node extends DetectableSceneNode
{
   private final PersistentDetection yoloDetection;

   // PersistentDetection values stored locally for syncing purposes
   private double confidence = 0.0;

   // YOLOv8Node specific variables
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private final Orientation3DBasics robotOrientation = new RotationMatrix();
   private final Pose3D objectPose;

   /**
    * Constructor used when the node does not have access to the persistent detection (e.g. UI side).
    * All values that would typically come from the persistent detection must be synced separately.
    */
   public YOLOv8Node(long id,
                     String name,
                     double confidence,
                     RigidBodyTransformReadOnly centroidToObjectTransform,
                     Pose3DReadOnly objectPose,
                     CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);

      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = new Pose3D(objectPose);
      this.confidence = confidence;
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
      Instant secondsAgo = Instant.now().minusSeconds(1);
      setCurrentlyDetected(mostRecentDetection.getDetectionTime().isAfter(secondsAgo) && yoloDetection.isStable());
      setConfidence(mostRecentDetection.getConfidence());

      objectPose.set(mostRecentDetection.getPose());
      objectPose.appendTransform(centroidToObjectTransform);
      objectPose.getRotation().set(robotOrientation);

      setNodeToParentFrameTransformAndUpdate(objectPose);
   }

   public void updateRobotOrientation(Orientation3DReadOnly orientation)
   {
      robotOrientation.set(orientation);
   }

   public RigidBodyTransformReadOnly getCentroidToObjectTransform()
   {
      return centroidToObjectTransform;
   }

   public double getConfidence()
   {
      return confidence;
   }

   public Pose3DReadOnly getObjectPose()
   {
      return objectPose;
   }

   public void toMessage(YOLOv8NodeMessage message)
   {
      message.setConfidence(getConfidence());
      message.getCentroidToObjectTransform().set(getCentroidToObjectTransform());
      message.getObjectPose().set(getObjectPose());
      message.getFilteredObjectPose().set(getObjectPose()); // FIXME Maybe set this to something else?
   }

   public void fromMessage(YOLOv8NodeMessage message)
   {
      setConfidence(message.getConfidence());
      setCentroidToObjectTransform(message.getCentroidToObjectTransform());
      setObjectPose(message.getObjectPose());
   }

   public void setConfidence(double confidence)
   {
      this.confidence = confidence;
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
