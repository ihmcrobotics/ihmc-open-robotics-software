package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Interface for saving and loading an action to file.
 */
public interface BehaviorActionData
{
   /** Needed to keep replaced reference frames up to date. */
   default void update()
   {

   }

   void saveToFile(ObjectNode jsonNode);

   void loadFromFile(JsonNode jsonNode) throws Exception;

   /**
    * A description of the action to help the operator in understanding
    * the purpose and context of the action.
    */
   String getDescription();

   /**
    * See {@link #getDescription()}.
    */
   void setDescription(String description);
}
