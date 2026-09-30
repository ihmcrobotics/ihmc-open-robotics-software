package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.FollowActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalString;

public class FollowActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalString targetLabel;
   private final CRDTBidirectionalBoolean closestPerson;

   private String onDiskTargetLabel;
   private boolean onDiskClosestPerson;

   public FollowActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      targetLabel = new CRDTBidirectionalString(this, "person");
      closestPerson = new CRDTBidirectionalBoolean(this, false);
      onDiskTargetLabel = "person";
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("targetLabel", targetLabel.getValue());
      jsonNode.put("closestPerson", closestPerson.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("targetLabel"))
         targetLabel.setValue(jsonNode.get("targetLabel").textValue());
      else
         targetLabel.setValue("person");

      if (jsonNode.has("closestPerson"))
         closestPerson.setValue(jsonNode.get("closestPerson").asBoolean());
      else
         closestPerson.setValue(false);
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskTargetLabel = targetLabel.getValue();
      onDiskClosestPerson = closestPerson.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         targetLabel.setValue(onDiskTargetLabel);
         closestPerson.setValue(onDiskClosestPerson);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= targetLabel.getValue().equals(onDiskTargetLabel);
      unchanged &= closestPerson.getValue() == onDiskClosestPerson;

      return !unchanged;
   }

   public void toMessage(FollowActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setTargetLabel(targetLabel.toMessage());
      message.setClosestPerson(closestPerson.toMessage());
   }

   public void fromMessage(FollowActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      targetLabel.fromMessage(message.getTargetLabelAsString());
      closestPerson.fromMessage(message.getClosestPerson());
   }

   public String getTargetLabel()
   {
      return targetLabel.getValue();
   }

   public void setTargetLabel(String targetLabel)
   {
      this.targetLabel.setValue(targetLabel);
   }

   public boolean getClosestPerson()
   {
      return closestPerson.getValue();
   }

   public void setClosestPerson(boolean closestPerson)
   {
      this.closestPerson.setValue(closestPerson);
   }
}
