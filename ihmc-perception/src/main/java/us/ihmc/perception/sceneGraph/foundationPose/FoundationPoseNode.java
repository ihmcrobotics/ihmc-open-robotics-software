package us.ihmc.perception.sceneGraph.foundationPose;

import perception_msgs.msg.dds.FoundationPoseNodeMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.FoundationPoseInstantDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;

import java.time.Instant;

public class FoundationPoseNode extends DetectableSceneNode
{
   private final PersistentDetection detection;
   private final Pose3D objectPose;

   public FoundationPoseNode(long id, String name, Pose3DReadOnly objectPose, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);

      this.objectPose = new Pose3D(objectPose);
      detection = null;
   }

   public FoundationPoseNode(long id, String name, CRDTInfo crdtInfo, PersistentDetection detection)
   {
      super(id, name, crdtInfo);

      this.detection = detection;
      objectPose = new Pose3D();
   }

   @Override
   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      super.update(sceneGraph, modificationQueue);

      FoundationPoseInstantDetection mostRecentDetection = (FoundationPoseInstantDetection) detection.getMostRecentDetection();
      Instant secondAgo = Instant.now().minusSeconds(1);
      setCurrentlyDetected(mostRecentDetection.getDetectionTime().isAfter(secondAgo) && detection.isStable());

      objectPose.set(mostRecentDetection.getPose());
      setNodeToParentFrameTransformAndUpdate(objectPose);
   }

   public void toMessage(FoundationPoseNodeMessage message)
   {
      message.getObjectPose().set(objectPose);
   }

   public void fromMessage(FoundationPoseNodeMessage message)
   {
      objectPose.set(message.getObjectPose());
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);
      if (detection != null)
         detection.markForDeletion();
   }
}
