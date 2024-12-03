package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.FallbackNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class FallbackNodeDefinition extends BehaviorTreeNodeDefinition
{
   private final CRDTBidirectionalLong gotoActionID;
   /** We use this to save the action name to file instead of the number for human readability. */
   @Nullable private String gotoActionName = null;

   // On disk fields
   private String onDiskGotoActionName;

   public FallbackNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      gotoActionID = new CRDTBidirectionalLong(this, 0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      if (gotoActionName != null)
         jsonNode.put("gotoActionName", gotoActionName);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("gotoAction"))
         gotoActionName = jsonNode.get("gotoAction").textValue();
      gotoActionID.setValue(0); // Invalidate until we can find it
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskGotoActionName = gotoActionName;
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      gotoActionName = onDiskGotoActionName;
      gotoActionID.setValue(0); // Invalidate until we can find it
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      if (gotoActionName != null)
         unchanged &= gotoActionName.equals(onDiskGotoActionName);

      return !unchanged;
   }

   public void toMessage(FallbackNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setGotoActionId(gotoActionID.toMessage());
   }

   public void fromMessage(FallbackNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      gotoActionID.fromMessage(message.getGotoActionId());
   }

   public CRDTBidirectionalLong getGotoActionID()
   {
      return gotoActionID;
   }

   public void setGotoActionName(@Nullable String gotoActionName)
   {
      this.gotoActionName = gotoActionName;
   }

   /** Only used for finding the ID after loading */
   @Nullable public String getGotoActionName()
   {
      return gotoActionName;
   }
}
