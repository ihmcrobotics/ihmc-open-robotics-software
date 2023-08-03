package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Interface for saving and loading an action to file.
 */
public interface BehaviorActionData
{
   default void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {

   }

   void saveToFile(ObjectNode jsonNode);

   void loadFromFile(JsonNode jsonNode);

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
