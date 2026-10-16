package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.HighLevelStateActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

/** Requests a high level controller state, such as WALKING or RL_CONTROL. */
public class HighLevelStateActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalEnumField<HighLevelControllerName> highLevelControllerName;
   private final CRDTBidirectionalDouble timeout;

   // On disk fields
   private HighLevelControllerName onDiskHighLevelControllerName;
   private double onDiskTimeout;

   public HighLevelStateActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      highLevelControllerName = new CRDTBidirectionalEnumField<>(this, HighLevelControllerName.WALKING);
      timeout = new CRDTBidirectionalDouble(this, 5.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("highLevelControllerName", highLevelControllerName.getValue().name());
      jsonNode.put("timeout", timeout.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      highLevelControllerName.setValue(HighLevelControllerName.valueOf(jsonNode.get("highLevelControllerName").textValue()));
      timeout.setValue(jsonNode.get("timeout").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskHighLevelControllerName = highLevelControllerName.getValue();
      onDiskTimeout = timeout.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         highLevelControllerName.setValue(onDiskHighLevelControllerName);
         timeout.setValue(onDiskTimeout);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= highLevelControllerName.getValue() == onDiskHighLevelControllerName;
      unchanged &= timeout.getValue() == onDiskTimeout;

      return !unchanged;
   }

   public void toMessage(HighLevelStateActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setHighLevelControllerName(highLevelControllerName.toMessageOrdinal());
      message.setTimeout(timeout.toMessage());
   }

   public void fromMessage(HighLevelStateActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      highLevelControllerName.fromMessageOrdinal(message.getHighLevelControllerName(), HighLevelControllerName.values);
      timeout.fromMessage(message.getTimeout());
   }

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName.getValue();
   }

   public void setHighLevelControllerName(HighLevelControllerName highLevelControllerName)
   {
      this.highLevelControllerName.setValue(highLevelControllerName);
   }

   public double getTimeout()
   {
      return timeout.getValue();
   }

   public void setTimeout(double timeout)
   {
      this.timeout.setValue(timeout);
   }
}
