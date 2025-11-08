package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

public class BehaviorTreeSceneObjectState extends LatestTimestampModifiable
{
   private final long id;
   private final IsaacROSFoundationPoseObject objectType;
   protected final CRDTBidirectionalRigidBodyTransform transform;
   protected final ReferenceFrame referenceFrame;

   public BehaviorTreeSceneObjectState(long id, CRDTInfo crdtInfo, IsaacROSFoundationPoseObject objectType)
   {
      super(crdtInfo);

      this.id = id;
      this.objectType = objectType;

      transform = new CRDTBidirectionalRigidBodyTransform(this);
      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("%s_%d".formatted(objectType.name(), id),
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       transform.getValueReadOnly());
   }

   public String getName()
   {
      return objectType.titleCaseName;
   }

   public IsaacROSFoundationPoseObject getObjectType()
   {
      return objectType;
   }

   public void clearOffset()
   {

   }

   public void freeze()
   {

   }

   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      toMessage(message.getLatestModificationToData());
      message.setId(id);
      message.setObjectType(objectType.ordinal());
      transform.toMessage(message.getTransformToWorld());
   }

   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      // Needs to be done first to detect incoming modification
      fromMessage(message.getLatestModificationToData());

      if (id != message.getId())
         LogTools.error("IDs should match! {} != {}", id, message.getId());

      if (objectType.ordinal() != message.getObjectType())
         LogTools.error("Object types should match! {} != {}", objectType.ordinal(), message.getObjectType());

      transform.fromMessage(message.getTransformToWorld());
      referenceFrame.update();
   }

   public void destroy()
   {

   }

   public long getID()
   {
      return id;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return transform.getValueReadOnly();
   }
}
