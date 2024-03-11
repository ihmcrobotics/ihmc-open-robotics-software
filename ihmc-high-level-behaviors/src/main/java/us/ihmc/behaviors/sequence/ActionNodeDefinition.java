package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTUnidirectionalLong;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

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

   private final CRDTUnidirectionalBoolean executeAfterPrevious;
   private final CRDTUnidirectionalBoolean executeAfterBeginning;
   private final CRDTUnidirectionalLong executeAfterNodeID;
   /** We use this to save the action name to file instead of the number for human readability. */
   private String executeAfterActionName = EXECUTE_AFTER_PREVIOUS;

   // On disk fields
   private String onDiskExecuteAfterActionName;

   // TODO: Remove
   public boolean executeWithNext = false;
   public boolean hasExecuterAfter = false;

   public ActionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      executeAfterPrevious = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, true);
      executeAfterBeginning = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      executeAfterNodeID = new CRDTUnidirectionalLong(ROS2ActorDesignation.OPERATOR, crdtInfo, 0);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeAfterAction", executeAfterActionName);
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
            executeAfterActionName = jsonNode.get("executeAfterAction").textValue();

            executeAfterPrevious.setValue(executeAfterActionName.equals(EXECUTE_AFTER_PREVIOUS));
            executeAfterBeginning.setValue(executeAfterActionName.equals(EXECUTE_AFTER_BEGINNING));
            executeAfterNodeID.setValue(0); // Invalidate until we can find it
//      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskExecuteAfterActionName = executeAfterActionName;
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      executeAfterActionName = onDiskExecuteAfterActionName;
      executeAfterPrevious.setValue(onDiskExecuteAfterActionName.equals(EXECUTE_AFTER_PREVIOUS));
      executeAfterBeginning.setValue(onDiskExecuteAfterActionName.equals(EXECUTE_AFTER_BEGINNING));
      executeAfterNodeID.setValue(0); // Invalidate until we can find it
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= executeAfterActionName.equals(onDiskExecuteAfterActionName);

      return !unchanged;
   }

   public void toMessage(ActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setExecuteAfterPrevious(executeAfterPrevious.toMessage());
      message.setExecuteAfterBeginning(executeAfterBeginning.toMessage());
      message.setExecuteAfterNodeId(executeAfterNodeID.toMessage());
   }

   public void fromMessage(ActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      executeAfterPrevious.fromMessage(message.getExecuteAfterPrevious());
      executeAfterBeginning.fromMessage(message.getExecuteAfterBeginning());
      executeAfterNodeID.fromMessage(message.getExecuteAfterNodeId());
   }

   public CRDTUnidirectionalBoolean getExecuteAfterPrevious()
   {
      return executeAfterPrevious;
   }

   public CRDTUnidirectionalBoolean getExecuteAfterBeginning()
   {
      return executeAfterBeginning;
   }

   public CRDTUnidirectionalLong getExecuteAfterNodeID()
   {
      return executeAfterNodeID;
   }

   /** Needs to be updated every tick on the operator side only. */
   public void updateExecuteAfterActionName(@Nullable String executeAfterActionName)
   {
      if (executeAfterBeginning.getValue())
      {
         this.executeAfterActionName = EXECUTE_AFTER_BEGINNING;
      }
      else if (executeAfterActionName != null)
      {
         this.executeAfterActionName = executeAfterActionName;
      }
      else
      {
         executeAfterPrevious.setValue(true);
         this.executeAfterActionName = EXECUTE_AFTER_PREVIOUS;
      }
   }
}
