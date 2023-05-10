package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Interface for saving and loading an action to file.
 */
public interface BehaviorActionData
{
   void saveToFile(ObjectNode jsonNode);

   void loadFromFile(JsonNode jsonNode);
}
