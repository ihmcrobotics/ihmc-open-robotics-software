package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
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
   // TODO: Is every action concurrent-able?
   private final CRDTUnidirectionalBoolean executeWithNextAction;

   public ActionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      executeWithNextAction = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeWithNextAction", executeWithNextAction.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      JsonNode executeWithNextActionNode = jsonNode.get("executeWithNextAction");
      if (executeWithNextActionNode != null)
         executeWithNextAction.setValue(executeWithNextActionNode.asBoolean());
      else
         executeWithNextAction.setValue(false);
   }

   public void toMessage(ActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setExecuteWithNextAction(executeWithNextAction.toMessage());
   }

   public void fromMessage(ActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      executeWithNextAction.fromMessage(message.getExecuteWithNextAction());
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWithNextAction.setValue(executeWitNextAction);
   }

   public boolean getExecuteWithNextAction()
   {
      return executeWithNextAction.getValue();
   }
}
