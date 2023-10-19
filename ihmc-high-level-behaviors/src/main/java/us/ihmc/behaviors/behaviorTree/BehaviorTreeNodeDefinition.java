package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.ArrayList;
import java.util.List;

/**
 * The base definition of a behavior tree node is just a
 * human readable description and a list of children.
 */
public class BehaviorTreeNodeDefinition
{
   /** A human readable description of what the node does */
   private String description = "";
   /** Behavior tree children node definitions. */
   private final List<BehaviorTreeNodeDefinition> children = new ArrayList<>();

   /**
    * Saves the file recursively.
    */
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("type", getClass().getSimpleName());

      if (!description.isEmpty()) // No reason to write default description
         jsonNode.put("description", description);

      ArrayNode childrenArrayJsonNode = jsonNode.putArray("children");
      for (BehaviorTreeNodeDefinition child : children)
      {
         ObjectNode childJsonNode = childrenArrayJsonNode.addObject();
         child.saveToFile(childJsonNode);
      }
   }

   /**
    * Loads just this node's definition data. Not recursive
    * because higher level node builders are required.
    */
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
   }

   public void toMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      message.setDescription(description);
   }

   public void fromMessage(BehaviorTreeNodeDefinitionMessage message)
   {
      description = message.getDescriptionAsString();
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

   public List<BehaviorTreeNodeDefinition> getChildren()
   {
      return children;
   }
}
