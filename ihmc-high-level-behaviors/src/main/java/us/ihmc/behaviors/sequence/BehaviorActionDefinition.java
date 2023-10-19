package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;

/**
 * Interface for a definition of an action with
 * support for saving and loading an action to file.
 *
 * This data includes only the information that defines an action,
 * which does not include the runtime state of it whether inactive
 * or currently executing it. This is only the information that gets
 * saved to/from JSON.
 */
public class BehaviorActionDefinition extends BehaviorTreeNodeDefinition
{
   // TODO: Is every action concurrent-able?
   private boolean executeWitNextAction = false;

   public BehaviorActionDefinition()
   {
      // Declared to use IDE to check usages
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeWithNextAction", executeWitNextAction);
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      JsonNode executeWithNextActionNode = jsonNode.get("executeWithNextAction");
      if (executeWithNextActionNode != null)
         executeWitNextAction = executeWithNextActionNode.asBoolean();
      else
         executeWitNextAction = false;
   }

   public void toMessage(BehaviorActionDefinitionMessage message)
   {
      super.toMessage(message.getNodeDefinition());

      message.setExecuteWithNextAction(getExecuteWithNextAction());
   }

   public void fromMessage(BehaviorActionDefinitionMessage message)
   {
      super.fromMessage(message.getNodeDefinition());

      executeWitNextAction = message.getExecuteWithNextAction();
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWitNextAction = executeWitNextAction;
   }

   public boolean getExecuteWithNextAction()
   {
      return executeWitNextAction;
   }
}
