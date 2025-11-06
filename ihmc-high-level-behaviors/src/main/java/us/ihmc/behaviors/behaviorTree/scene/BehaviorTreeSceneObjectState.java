package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;

public class BehaviorTreeSceneObjectState extends LatestTimestampModifiable
{
   private final long id;
   private final String type;
   protected final CRDTBidirectionalRigidBodyTransform transform;
   protected final ReferenceFrame referenceFrame;

   public BehaviorTreeSceneObjectState(long id, CRDTInfo crdtInfo, String type)
   {
      super(crdtInfo);

      this.id = id;
      this.type = type;

      transform = new CRDTBidirectionalRigidBodyTransform(this);
      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("%s_%d".formatted(type, id),
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       transform.getValueReadOnly());
   }

   public String getName()
   {
      return type;
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
      message.setType(type);
      transform.toMessage(message.getTransformToWorld());
   }

   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      // Needs to be done first to detect incoming modification
      fromMessage(message.getLatestModificationToData());

      if (id != message.getId())
         LogTools.error("IDs should match! {} != {}", id, message.getId());

      if (!type.equals(message.getTypeAsString()))
         LogTools.error("Types should match! {} != {}", id, message.getId());

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
