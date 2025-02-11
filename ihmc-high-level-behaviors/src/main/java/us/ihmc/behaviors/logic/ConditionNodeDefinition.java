package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.LeafNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * The first version of this node just implements a counter.
 *
 * TODO: Extend such that conditions can be setup through an interface
 *   and registered as an option for the operator to select.
 */
public class ConditionNodeDefinition extends LeafNodeDefinition
{
   private final CRDTBidirectionalLong countTo;

   private long onDiskCountTo = 0;

   public ConditionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      countTo = new CRDTBidirectionalLong(this, 0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("countTo", countTo.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      countTo.setValue(jsonNode.get("countTo").longValue());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskCountTo = countTo.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         countTo.setValue(onDiskCountTo);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= countTo.getValue() == onDiskCountTo;

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setCountTo(countTo.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      countTo.fromMessage(message.getCountTo());
   }

   public CRDTBidirectionalLong getCountTo()
   {
      return countTo;
   }
}
