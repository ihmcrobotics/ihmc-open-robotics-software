package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.log.LogTools;
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

   private final CRDTBidirectionalLong executeAfterNodeID;
   /** We use this to save the action name to file instead of the number for human readability. */
   private String executeAfterActionName = EXECUTE_AFTER_PREVIOUS;

   // On disk fields
   private String onDiskExecuteAfterActionName;

   public ActionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      executeAfterNodeID = new CRDTBidirectionalLong(this, ActionNodeDefinitionMessage.EXECUTE_AFTER_PREVIOUS);
   }

   public void updateAndSanitizeExecuteAfterFields(@Nullable String executeAfterActionName)
   {
      if (executeAfterBeginning.getValue())
      {
         this.executeAfterActionName = EXECUTE_AFTER_BEGINNING;
         if (getName().contains("Wait"))
            LogTools.debug("{}: beginning {} -> 0", getCRDTInfo().getActorDesignation().name(), executeAfterNodeID.getValue());
         executeAfterNodeID.setValue(0);
      }
      else if (executeAfterActionName != null)
      {
         this.executeAfterActionName = executeAfterActionName;
      }
      else // Default to previous
      {
         executeAfterPrevious.setValue(true);
         this.executeAfterActionName = EXECUTE_AFTER_PREVIOUS;
         if (getName().contains("Wait") && executeAfterNodeID.getValue() != 0)
            LogTools.debug("{}: Defaulting to previous {} -> 0", getCRDTInfo().getActorDesignation().name(), executeAfterNodeID.getValue());
         executeAfterNodeID.setValue(0);
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("executeAfterAction", executeAfterActionName);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      executeAfterActionName = jsonNode.get("executeAfterAction").textValue();
      if (getName().contains("Wait"))
         LogTools.debug("Loaded {}", executeAfterActionName);
      executeAfterPrevious.setValue(executeAfterActionName.equals(EXECUTE_AFTER_PREVIOUS));
      executeAfterBeginning.setValue(executeAfterActionName.equals(EXECUTE_AFTER_BEGINNING));
      executeAfterNodeID.setValue(0); // Invalidate until we can find it
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

      if (isUndoAvailable())
      {
         executeAfterActionName = onDiskExecuteAfterActionName;
         executeAfterNodeID.setValue(0); // Invalidate until we can find it
      }
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

      if (getName().contains("Wait") && executeAfterNodeID.getValue() != message.getExecuteAfterNodeId())
         LogTools.debug("{}: toMessage {} -> {}", getCRDTInfo().getActorDesignation().name(), message.getExecuteAfterNodeId(),
                        executeAfterNodeID.getValue());
      message.setExecuteAfterNodeId(executeAfterNodeID.toMessage());
   }

   public void fromMessage(ActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      if (getName().contains("Wait") && executeAfterNodeID.getValue() != message.getExecuteAfterNodeId())
         LogTools.debug("{}: fromMessage {} -> {}", getCRDTInfo().getActorDesignation().name(), executeAfterNodeID.getValue(), message.getExecuteAfterNodeId());
      executeAfterNodeID.fromMessage(message.getExecuteAfterNodeId());
   }

   public boolean getExecuteAfterPrevious()
   {
      return executeAfterNodeID.getValue() == ActionNodeDefinitionMessage.EXECUTE_AFTER_PREVIOUS;
   }

   public boolean getExecuteAfterBeginning()
   {
      return executeAfterNodeID.getValue() == ActionNodeDefinitionMessage.EXECUTE_AFTER_BEGINNING;
   }

   public CRDTBidirectionalLong getExecuteAfterNodeID()
   {
      return executeAfterNodeID;
   }

   /** Only used for finding the ID after loading */
   public String getExecuteAfterActionName()
   {
      return executeAfterActionName;
   }
}
