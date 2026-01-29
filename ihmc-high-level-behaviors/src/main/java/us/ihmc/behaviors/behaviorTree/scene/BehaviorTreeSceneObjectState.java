package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;

public class BehaviorTreeSceneObjectState extends BehaviorTreeSceneObjectDefinition
{
   private final long id;
   protected final CRDTBidirectionalRigidBodyTransform transform;
   protected final ReferenceFrame referenceFrame;

   public BehaviorTreeSceneObjectState(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(crdtInfo, definition);

      this.id = id;

      transform = new CRDTBidirectionalRigidBodyTransform(this);
      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("%s_%d".formatted(getName(), id),
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       transform.getValueReadOnly());
   }

   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message.getLatestModificationToData());
      message.setId(id);
      super.toMessage(message.getDefinition());
      transform.toMessage(message.getTransformToWorld());
   }

   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      // Needs to be done first to detect incoming modification
      fromMessage(message.getLatestModificationToData());

      if (id != message.getId())
         LogTools.error("IDs should match! {} != {}", id, message.getId());

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
