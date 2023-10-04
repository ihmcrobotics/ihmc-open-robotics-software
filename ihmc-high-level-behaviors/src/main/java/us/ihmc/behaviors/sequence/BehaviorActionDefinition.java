package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Interface for a definition of an action with
 * support for saving and loading an action to file.
 *
 * This data includes only the information that defines an action,
 * which does not include the runtime state of it whether inactive
 * or currently executing it. This is only the information that gets
 * saved to/from JSON.
 */
public interface BehaviorActionDefinition
{
   default void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {

   }

   /** Needed to keep replaced reference frames up to date. */
   default void update()
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

   default boolean getExecuteWithNextAction()
   {
      return false;
   }
}
