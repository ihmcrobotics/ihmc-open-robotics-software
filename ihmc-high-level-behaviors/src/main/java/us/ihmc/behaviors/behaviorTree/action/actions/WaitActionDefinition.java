package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.WaitActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;

public class WaitActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalDouble waitDuration;

   // On disk fields
   private double onDiskWaitDuration;

   public WaitActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      waitDuration = new CRDTBidirectionalDouble(this, 4.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("waitDuration", waitDuration.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      waitDuration.setValue(jsonNode.get("waitDuration").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskWaitDuration = waitDuration.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         waitDuration.setValue(onDiskWaitDuration);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= waitDuration.getValue() == onDiskWaitDuration;

      return !unchanged;
   }

   public void toMessage(WaitActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setWaitDuration(waitDuration.toMessage());
   }

   public void fromMessage(WaitActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      waitDuration.fromMessage(message.getWaitDuration());
   }

   public double getWaitDuration()
   {
      return waitDuration.getValue();
   }

   public void setWaitDuration(double waitDuration)
   {
      this.waitDuration.setValue(waitDuration);
   }
}
