package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

public class SceneActionNodeDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalEnumField<IsaacROSFoundationPoseObject> objectType;

   private IsaacROSFoundationPoseObject onDiskObjectType;

   public SceneActionNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      objectType = new CRDTBidirectionalEnumField<>(this, IsaacROSFoundationPoseObject.MUSTARD);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("objectType", objectType.getValue().name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      objectType.setValue(IsaacROSFoundationPoseObject.valueOf(jsonNode.get("objectType").asText()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskObjectType = objectType.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         objectType.setValue(onDiskObjectType);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= objectType.getValue() == onDiskObjectType;

      return !unchanged;
   }

   public void toMessage(SceneActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setObjectType(objectType.toMessageOrdinal());
   }

   public void fromMessage(SceneActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      objectType.fromMessageOrdinal(message.getObjectType(), IsaacROSFoundationPoseObject.values);
   }

   public IsaacROSFoundationPoseObject getObjectType()
   {
      return objectType.getValue();
   }

   public void setObjectType(IsaacROSFoundationPoseObject objectType)
   {
      this.objectType.setValue(objectType);
   }
}
