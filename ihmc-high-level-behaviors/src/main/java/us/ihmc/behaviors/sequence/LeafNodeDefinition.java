package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.LeafNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

/**
 * Definition of a leaf node which gets executed as part of the concurrency system.
 *
 * This is used for action, condition, and goto nodes.
 */
public class LeafNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final String EXECUTE_AFTER_PREVIOUS = "Previous";
   public static final String EXECUTE_AFTER_BEGINNING = "Beginning";

   private final CRDTBidirectionalBoolean executeAfterPrevious;
   private final CRDTBidirectionalBoolean executeAfterBeginning;
   private final CRDTBidirectionalLong executeAfterNodeID;
   /** We use this to save the action name to file instead of the number for human readability. */
   private String executeAfterActionName = EXECUTE_AFTER_PREVIOUS;

   // On disk fields
   private String onDiskExecuteAfterActionName;

   public LeafNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      executeAfterPrevious = new CRDTBidirectionalBoolean(this, true);
      executeAfterBeginning = new CRDTBidirectionalBoolean(this, false);
      executeAfterNodeID = new CRDTBidirectionalLong(this, 0);
   }

   public void updateAndSanitizeExecuteAfterFields(@Nullable String executeAfterActionName)
   {
      if (executeAfterBeginning.getValue())
      {
         this.executeAfterActionName = EXECUTE_AFTER_BEGINNING;
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

   public void toMessage(LeafNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setExecuteAfterPrevious(executeAfterPrevious.toMessage());
      message.setExecuteAfterBeginning(executeAfterBeginning.toMessage());
      message.setExecuteAfterNodeId(executeAfterNodeID.toMessage());
   }

   public void fromMessage(LeafNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      executeAfterPrevious.fromMessage(message.getExecuteAfterPrevious());
      executeAfterBeginning.fromMessage(message.getExecuteAfterBeginning());
      executeAfterNodeID.fromMessage(message.getExecuteAfterNodeId());
   }

   public CRDTBidirectionalBoolean getExecuteAfterPrevious()
   {
      return executeAfterPrevious;
   }

   public CRDTBidirectionalBoolean getExecuteAfterBeginning()
   {
      return executeAfterBeginning;
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
