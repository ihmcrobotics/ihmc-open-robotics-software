package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Interface for a definition of an action with
 * support for saving and loading an action to file.
 *
 * This data includes only the information that defines an action,
 * which does not include the runtime state of it whether inactive
 * or currently executing it. This is only the information that gets
 * saved to/from JSON.
 */
public class BehaviorActionDefinition
{
   /** Human readable description of what the action does */
   private String description;
   // TODO: Is every action concurrent-able?
   private boolean executeWitNextAction = false;

   public BehaviorActionDefinition(String description)
   {
      this.description = description;
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("executeWithNextAction", executeWitNextAction);
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      JsonNode executeWithNextActionNode = jsonNode.get("executeWithNextAction");
      if (executeWithNextActionNode != null)
         executeWitNextAction = executeWithNextActionNode.asBoolean();
      else
         executeWitNextAction = false;
   }

   public void toMessage(BehaviorActionDefinitionMessage message)
   {
      message.setDescription(description);
      message.setExecuteWithNextAction(getExecuteWithNextAction());
   }

   public void fromMessage(BehaviorActionDefinitionMessage message)
   {
      description = message.getDescriptionAsString();
      executeWitNextAction = message.getExecuteWithNextAction();
   }

   /**
    * A description of the action to help the operator in understanding
    * the purpose and context of the action.
    */
   public void setDescription(String description)
   {
      this.description = description;
   }

   public String getDescription()
   {
      return description;
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
