package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
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

   // TODO: Is every action concurrent-able?
   private final CRDTUnidirectionalString executeAfterAction;

   // On disk fields
   private String onDiskExecuteAfterAction;

   public boolean executeWithNext = false;

   public ActionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      executeAfterAction = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, EXECUTE_AFTER_PREVIOUS);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeAfterAction", executeAfterAction.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      JsonNode executeWithNextActionNode = jsonNode.get("executeWithNextAction");
      if (executeWithNextActionNode != null)
         executeWithNext = executeWithNextActionNode.asBoolean();
//      if (executeWithNextActionNode.asBoolean())
//      {
//         executeAfterAction.setValue(EXECUTE_AFTER_DEFAULT);
//      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskExecuteAfterAction = executeAfterAction.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      executeAfterAction.setValue(onDiskExecuteAfterAction);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= executeAfterAction.getValue().equals(onDiskExecuteAfterAction);

      return !unchanged;
   }

   public void toMessage(ActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setExecuteAfterAction(executeAfterAction.toMessage());
   }

   public void fromMessage(ActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      executeAfterAction.fromMessage(message.getExecuteAfterActionAsString());
   }

   public void setExecuteAfterAction(String executeAfterAction)
   {
      this.executeAfterAction.setValue(executeAfterAction);
   }

   public String getExecuteAfterAction()
   {
      return executeAfterAction.getValue();
   }
}
