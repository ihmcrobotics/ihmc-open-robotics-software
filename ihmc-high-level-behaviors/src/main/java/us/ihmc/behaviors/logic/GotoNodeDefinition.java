package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.GotoNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class GotoNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final String GOTO_NEXT = "Next";

   private final CRDTBidirectionalBoolean gotoNext;
   private final CRDTBidirectionalLong gotoNodeID;
   /** We use this to save the goto node name to file instead of the number for human readability. */
   private String gotoNodeName = GOTO_NEXT;

   // On disk fields
   private String onDiskGotoNodeName;

   public GotoNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      gotoNext = new CRDTBidirectionalBoolean(this, true);
      gotoNodeID = new CRDTBidirectionalLong(this, 0);
   }

   public void updateAndSanitizeGotoNodeFields(@Nullable String gotoNodeName)
   {
      if (gotoNodeName != null)
      {
         this.gotoNodeName = gotoNodeName;
      }
      else // Default to next
      {
         gotoNext.setValue(true);
         this.gotoNodeName = GOTO_NEXT;
         gotoNodeID.setValue(0);
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("gotoNode", gotoNodeName);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      gotoNodeName = jsonNode.get("gotoNode").textValue();
      gotoNext.setValue(gotoNodeName.equals(GOTO_NEXT));
      gotoNodeID.setValue(0); // Invalidate until we can find it
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskGotoNodeName = gotoNodeName;
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      gotoNodeName = onDiskGotoNodeName;
      gotoNext.setValue(onDiskGotoNodeName.equals(GOTO_NEXT));
      gotoNodeID.setValue(0); // Invalidate until we can find it
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= gotoNodeName.equals(onDiskGotoNodeName);

      return !unchanged;
   }

   public void toMessage(GotoNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setGotoNext(gotoNext.toMessage());
      message.setGotoNodeId(gotoNodeID.toMessage());
   }

   public void fromMessage(GotoNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      gotoNext.fromMessage(message.getGotoNext());
      gotoNodeID.fromMessage(message.getGotoNodeId());
   }

   public CRDTBidirectionalBoolean getGotoNext()
   {
      return gotoNext;
   }

   public CRDTBidirectionalLong getGotoNodeID()
   {
      return gotoNodeID;
   }

   /** Only used for finding the ID after loading */
   public String getGotoNodeName()
   {
      return gotoNodeName;
   }
}
