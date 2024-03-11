package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * Interface for a definition of an action with
 * support for saving and loading an action to file.
 *
 * This data includes only the information that defines an action,
 * which does not include the runtime state of it whether inactive
 * or currently executing it. This is only the information that gets
 * saved to/from JSON.
 */
public class ActionNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final String EXECUTE_AFTER_PREVIOUS = "Previous";
   public static final String EXECUTE_AFTER_BEGINNING = "Beginning";

   private CRDTUnidirectionalInteger distanceToExecuteAfter;
   /** We use this to save the node name to file instead of the number for human readability. */
   private String executeAfterNodeName = EXECUTE_AFTER_PREVIOUS;

   // On disk fields
   private String onDiskExecuteAfterAction;

   // TODO: Remove
   public boolean executeWithNext = false;
   public boolean hasExecuterAfter = false;

   public ActionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      distanceToExecuteAfter = new CRDTUnidirectionalInteger(ROS2ActorDesignation.OPERATOR, crdtInfo, 1); // Default to previous
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeAfterAction", distanceToExecuteAfter.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

//      if (jsonNode.get("executeWithNextAction") != null)
//         executeWithNext = jsonNode.get("executeWithNextAction").asBoolean();
//
//      if (jsonNode.get("executeAfterAction") != null)
//      {
//         hasExecuterAfter = true;
         executeAfterNodeName = jsonNode.get("executeAfterAction").textValue();
         distanceToExecuteAfter.setValue(0); // Invalidate until we can find it

//      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskExecuteAfterAction = executeAfterNodeName;
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      executeAfterNodeName = onDiskExecuteAfterAction;
      distanceToExecuteAfter.setValue(0); // Invalidate until we can find it
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= executeAfterNodeName.equals(onDiskExecuteAfterAction);

      return !unchanged;
   }

   public void toMessage(ActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setDistanceToExecuteAfter(distanceToExecuteAfter.toMessage());
   }

   public void fromMessage(ActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      distanceToExecuteAfter.fromMessage(message.getDistanceToExecuteAfter());
   }

   public CRDTUnidirectionalInteger getDistanceToExecuteAfter()
   {
      return distanceToExecuteAfter;
   }

   public void setExecuteAfterNodeName(String executeAfterNodeName)
   {
      this.executeAfterNodeName = executeAfterNodeName;
   }
}
