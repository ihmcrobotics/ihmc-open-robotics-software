package us.ihmc.perception.sceneGraph.rigidBody.trashcan;

import perception_msgs.msg.dds.TrashCanNodeMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class TrashCanNode extends DetectableSceneNode
{
   // Sent over ROS2
   private final RigidBodyTransform trashCanToWorldTransform = new RigidBodyTransform();
   private double trashCanYaw = 0.0;

   // Not sent over ROS2
   private final PersistentDetection trashCanDetection;
   private final MutableReferenceFrame trashCanFrame = new MutableReferenceFrame();

   public TrashCanNode(long id, String name, CRDTInfo crdtInfo)
   {
      this(id, name, null, crdtInfo);
   }

   public TrashCanNode(long id, String name, PersistentDetection trashCanDetection, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      this.trashCanDetection = trashCanDetection;

      // Initially set everything to NaN
      trashCanToWorldTransform.setToNaN();
      trashCanFrame.update(transformToWorld -> transformToWorld.set(trashCanToWorldTransform));
   }

   // on robot only
   @Override
   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      super.update(sceneGraph, modificationQueue);

      setCurrentlyDetected(trashCanDetection.isStable());

      Point3DReadOnly trashCanCentroid = trashCanDetection.getMostRecentPosition();

      // Update the transforms to world
      trashCanToWorldTransform.setTranslationAndIdentityRotation(trashCanCentroid);
      trashCanToWorldTransform.getRotation().setYawPitchRoll(trashCanYaw, 0.0, 0.0);

      // Update the frames
      trashCanFrame.update(transformToWorld -> transformToWorld.set(trashCanToWorldTransform));

      // Update node frame
      getNodeToParentFrameTransform().set(trashCanFrame.getTransformToParent());
      getNodeFrame().update();
   }

   public void updateFromMessage(TrashCanNodeMessage message)
   {
      trashCanYaw = message.getTrashCanYaw();
      trashCanToWorldTransform.set(message.getTrashCanToWorldTransform());
      trashCanToWorldTransform.getRotation().setYawPitchRoll(trashCanYaw, 0.0, 0.0);
      trashCanFrame.update(transformToWorld -> transformToWorld.set(trashCanToWorldTransform));
   }

   public double getYaw()
   {
      return trashCanYaw;
   }

   public void setYaw(double yaw)
   {
      this.trashCanYaw = yaw;
   }

   public RigidBodyTransformReadOnly getTrashCanToWorldTransform()
   {
      return trashCanToWorldTransform;
   }

   public ReferenceFrame getTrashCanFrame()
   {
      return trashCanFrame.getReferenceFrame();
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);

      if (trashCanDetection != null)
         trashCanDetection.markForDeletion();
   }
}
