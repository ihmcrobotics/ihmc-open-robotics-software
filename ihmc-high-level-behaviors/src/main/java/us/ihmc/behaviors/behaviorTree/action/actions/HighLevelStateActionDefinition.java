package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.HighLevelStateActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

/** Requests a high level controller state, such as WALKING or RL_CONTROL, and with RL_CONTROL, optionally an RL model. */
public class HighLevelStateActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalEnumField<HighLevelControllerName> highLevelControllerName;
   private final CRDTBidirectionalDouble timeout;
   /** Name in RLPolicyState's available models. Empty keeps the current model. Only used with RL_CONTROL. */
   private final CRDTBidirectionalString rlModelName;
   /** Hold the arms, spine and neck where they are in the new state instead of moving them to its home pose */
   private final CRDTBidirectionalBoolean keepUpperBodyPose;

   // On disk fields
   private HighLevelControllerName onDiskHighLevelControllerName;
   private double onDiskTimeout;
   private String onDiskRLModelName;
   private boolean onDiskKeepUpperBodyPose;

   public HighLevelStateActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      highLevelControllerName = new CRDTBidirectionalEnumField<>(this, HighLevelControllerName.WALKING);
      timeout = new CRDTBidirectionalDouble(this, 5.0);
      rlModelName = new CRDTBidirectionalString(this, "");
      keepUpperBodyPose = new CRDTBidirectionalBoolean(this, false);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("highLevelControllerName", highLevelControllerName.getValue().name());
      jsonNode.put("timeout", timeout.getValue());
      jsonNode.put("rlModelName", rlModelName.getValue());
      jsonNode.put("keepUpperBodyPose", keepUpperBodyPose.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      highLevelControllerName.setValue(HighLevelControllerName.valueOf(jsonNode.get("highLevelControllerName").textValue()));
      timeout.setValue(jsonNode.get("timeout").asDouble());
      rlModelName.setValue(jsonNode.has("rlModelName") ? jsonNode.get("rlModelName").textValue() : "");
      keepUpperBodyPose.setValue(jsonNode.has("keepUpperBodyPose") && jsonNode.get("keepUpperBodyPose").asBoolean());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskHighLevelControllerName = highLevelControllerName.getValue();
      onDiskTimeout = timeout.getValue();
      onDiskRLModelName = rlModelName.getValue();
      onDiskKeepUpperBodyPose = keepUpperBodyPose.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         highLevelControllerName.setValue(onDiskHighLevelControllerName);
         timeout.setValue(onDiskTimeout);
         rlModelName.setValue(onDiskRLModelName);
         keepUpperBodyPose.setValue(onDiskKeepUpperBodyPose);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= highLevelControllerName.getValue() == onDiskHighLevelControllerName;
      unchanged &= timeout.getValue() == onDiskTimeout;
      unchanged &= rlModelName.getValue().equals(onDiskRLModelName);
      unchanged &= keepUpperBodyPose.getValue() == onDiskKeepUpperBodyPose;

      return !unchanged;
   }

   public void toMessage(HighLevelStateActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setHighLevelControllerName(highLevelControllerName.toMessageOrdinal());
      message.setTimeout(timeout.toMessage());
      message.setRlModelName(rlModelName.toMessage());
      message.setKeepUpperBodyPose(keepUpperBodyPose.toMessage());
   }

   public void fromMessage(HighLevelStateActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      highLevelControllerName.fromMessageOrdinal(message.getHighLevelControllerName(), HighLevelControllerName.values);
      timeout.fromMessage(message.getTimeout());
      rlModelName.fromMessage(message.getRlModelNameAsString());
      keepUpperBodyPose.fromMessage(message.getKeepUpperBodyPose());
   }

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName.getValue();
   }

   public void setHighLevelControllerName(HighLevelControllerName highLevelControllerName)
   {
      this.highLevelControllerName.setValue(highLevelControllerName);
   }

   public double getTimeout()
   {
      return timeout.getValue();
   }

   public void setTimeout(double timeout)
   {
      this.timeout.setValue(timeout);
   }

   public String getRLModelName()
   {
      return rlModelName.getValue();
   }

   public void setRLModelName(String rlModelName)
   {
      this.rlModelName.setValue(rlModelName);
   }

   public boolean getKeepUpperBodyPose()
   {
      return keepUpperBodyPose.getValue();
   }

   public void setKeepUpperBodyPose(boolean keepUpperBodyPose)
   {
      this.keepUpperBodyPose.setValue(keepUpperBodyPose);
   }
}
