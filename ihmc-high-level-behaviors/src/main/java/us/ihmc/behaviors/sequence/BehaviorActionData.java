package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

public interface BehaviorActionData
{
   void saveToFile(ObjectNode jsonNode);

   void loadFromFile(JsonNode jsonNode);
}
