package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;

public class BehaviorTreeSceneObjectState extends BehaviorTreeSceneObjectDefinition
{
   private final long id;
   protected final CRDTBidirectionalBoolean valid;
   protected final CRDTBidirectionalBoolean frozen;
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
      valid = new CRDTBidirectionalBoolean(this, false);
      frozen = new CRDTBidirectionalBoolean(this, false);
   }

   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message.getLatestModificationToData());
      message.setId((int) id);
      super.toMessage(message.getDefinition());
      transform.toMessage(message.getTransformToWorld());
      message.setValid(valid.toMessage());
      message.setFrozen(frozen.toMessage());
   }

   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      // Needs to be done first to detect incoming modification
      fromMessage(message.getLatestModificationToData());

      if (id != message.getId())
         LogTools.error("IDs should match! {} != {}", id, message.getId());

      fromMessage(message.getDefinition());

      transform.fromMessage(message.getTransformToWorld());
      referenceFrame.update();
      valid.fromMessage(message.getValid());
      frozen.fromMessage(message.getFrozen());
   }

   public void destroy()
   {

   }

   public void freeze()
   {
      frozen.setValue(true);
   }

   public void unfreeze()
   {
      frozen.setValue(false);
   }

   public boolean isFrozen()
   {
      return frozen.getValue();
   }

   public void setValid(boolean valid)
   {
      this.valid.setValue(valid);
   }

   public boolean isValid()
   {
      return valid.getValue();
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

   public void setTransformToWorld(RigidBodyTransformReadOnly transformToWorld)
   {
      transform.getValueAndModify().set(transformToWorld);
      referenceFrame.update();
      valid.setValue(true);
   }
}
