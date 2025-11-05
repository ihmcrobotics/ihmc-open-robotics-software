package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.handsros2.abilityHand.AbilityHandManager;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;

public class AbilityHandActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTBidirectionalEnumField<RobotSide> side;
   private final CRDTBidirectionalEnumField<AbilityHandManager.Grip> grip;

   // On disk fields
   private RobotSide onDiskSide;
   private AbilityHandManager.Grip onDiskGrip;

   public AbilityHandActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      side = new CRDTBidirectionalEnumField<>(this, RobotSide.LEFT);
      grip = new CRDTBidirectionalEnumField<>(this, Grip.PINCH_O);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("grip", grip.getValue().name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      grip.setValue(AbilityHandManager.Grip.valueOf(jsonNode.get("grip").asText()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskGrip = grip.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         side.setValue(onDiskSide);
         grip.setValue(onDiskGrip);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= grip.getValue() == onDiskGrip;

      return !unchanged;
   }

   public void toMessage(AbilityHandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setGrip(grip.toMessage().toByte());
   }

   public void fromMessage(AbilityHandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      grip.fromMessageOrdinal(message.getGrip(), Grip.values);
   }

   @Override
   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public AbilityHandManager.Grip getGrip()
   {
      return grip.getValue();
   }

   public void setGrip(AbilityHandManager.Grip grip)
   {
      this.grip.setValue(grip);
   }
}
