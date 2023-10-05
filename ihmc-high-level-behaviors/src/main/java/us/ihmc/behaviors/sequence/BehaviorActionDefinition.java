package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.packets.Packet;

/**
 * Interface for a definition of an action with
 * support for saving and loading an action to file.
 *
 * This data includes only the information that defines an action,
 * which does not include the runtime state of it whether inactive
 * or currently executing it. This is only the information that gets
 * saved to/from JSON.
 */
public abstract class BehaviorActionDefinition<T extends Packet<T>>
{
   private String description;
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

   public abstract void toMessage(T message);

   public abstract void fromMessage(T message);

   /**
    * A description of the action to help the operator in understanding
    * the purpose and context of the action.
    */
   public void setDescription(String description)
   {
      this.description = description;
   }

   /**
    * See {@link #getDescription()}.
    */
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
