package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripSpeed;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripType;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PsyonicAbilityHandCommandActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalEnumField<LegacyGripType> legacyGripType;
   private final CRDTUnidirectionalEnumField<LegacyGripSpeed> legacyGripSpeed;

   // On disk fields
   private RobotSide onDiskSide;
   private LegacyGripType onDiskLegacyGripType;
   private LegacyGripSpeed onDiskLegacyGripSpeed;

   public PsyonicAbilityHandCommandActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      legacyGripType = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, LegacyGripType.MOUSE_GRIP);
      legacyGripSpeed = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, LegacyGripSpeed.MEDIUM);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("legacyGripType", legacyGripType.getValue().name());
      jsonNode.put("legacyGripSpeed", legacyGripSpeed.getValue().name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));

      setLegacyGripType(LegacyGripType.valueOf(jsonNode.get("legacyGripType").textValue()));
      setLegacyGripSpeed(LegacyGripSpeed.valueOf(jsonNode.get("legacyGripSpeed").textValue()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskLegacyGripType = LegacyGripType.valueOf(legacyGripType.getValue().name());
      onDiskLegacyGripSpeed = LegacyGripSpeed.valueOf(legacyGripSpeed.getValue().name());
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      setLegacyGripType(LegacyGripType.valueOf(onDiskLegacyGripType.name()));
      setLegacyGripSpeed(LegacyGripSpeed.valueOf(onDiskLegacyGripSpeed.name()));
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= legacyGripType.getValue() == onDiskLegacyGripType;
      unchanged &= legacyGripSpeed.getValue() == onDiskLegacyGripSpeed;

      return !unchanged;
   }

   public void toMessage(PsyonicAbilityHandCommandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setGripType(legacyGripType.getValue().name());
      message.setGripSpeed(legacyGripSpeed.getValue().name());
   }

   public void fromMessage(PsyonicAbilityHandCommandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      legacyGripType.setValue(LegacyGripType.valueOf(message.getGripTypeAsString()));
      legacyGripSpeed.setValue(LegacyGripSpeed.valueOf(message.getGripSpeedAsString()));
   }

   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public LegacyGripType getLegacyGripType()
   {
      return legacyGripType.getValue();
   }

   public void setLegacyGripType(LegacyGripType legacyGripType)
   {
      this.legacyGripType.setValue(legacyGripType);
   }

   public LegacyGripSpeed getLegacyGripSpeed()
   {
      return legacyGripSpeed.getValue();
   }

   public void setLegacyGripSpeed(LegacyGripSpeed legacyGripSpeed)
   {
      this.legacyGripSpeed.setValue(legacyGripSpeed);
   }
}
