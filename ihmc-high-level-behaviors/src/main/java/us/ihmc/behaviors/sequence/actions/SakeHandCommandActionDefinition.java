package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.sakeGripper.SakeHandPresets;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTUnidirectionalInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class SakeHandCommandActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalInteger handConfigurationIndex;
   private final CRDTUnidirectionalDouble handOpenAngle;
   private final CRDTUnidirectionalDouble maxTorque;

   public SakeHandCommandActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      handConfigurationIndex = new CRDTUnidirectionalInteger(ROS2ActorDesignation.OPERATOR, crdtInfo, SakeHandPresets.GOTO.ordinal());
      handOpenAngle = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0); // default to open
      maxTorque = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0); // default to none
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("configuration", SakeHandPresets.values[handConfigurationIndex.getValue()].name());
      jsonNode.put("handOpenAngle", handOpenAngle.getValue());
      jsonNode.put("maxTorque", maxTorque.getValue());
//      jsonNode.put("position", handOpenAngle.getValue());
//      jsonNode.put("torque", maxTorque.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      handConfigurationIndex.setValue(SakeHandPresets.valueOf(jsonNode.get("configuration").asText()).ordinal());


//      handOpenAngle.setValue(jsonNode.get("handOpenAngle").asDouble());
//      maxTorque.setValue(jsonNode.get("maxTorque").asDouble());
      handOpenAngle.setValue(jsonNode.get("position").asDouble());
      maxTorque.setValue(jsonNode.get("torque").asDouble());
   }

   public void toMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setConfiguration(handConfigurationIndex.toMessage());
      message.setPositionRatio(handOpenAngle.toMessage());
      message.setTorqueRatio(maxTorque.toMessage());
   }

   public void fromMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      handConfigurationIndex.fromMessage((int) message.getConfiguration());
      handOpenAngle.fromMessage(message.getPositionRatio());
      maxTorque.fromMessage(message.getTorqueRatio());
   }

   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public int getHandConfigurationIndex()
   {
      return handConfigurationIndex.getValue();
   }

   public SakeHandPresets getSakeCommandOption()
   {
      return SakeHandPresets.values[handConfigurationIndex.getValue()];
   }

   public void setHandConfigurationIndex(int handConfigurationIndex)
   {
      this.handConfigurationIndex.setValue(handConfigurationIndex);
   }

   public double getHandOpenAngle()
   {
      return handOpenAngle.getValue();
   }

   public double getMaxTorque()
   {
      return maxTorque.getValue();
   }

   public void setHandOpenAngle(double handOpenAngle)
   {
      this.handOpenAngle.setValue(handOpenAngle);
   }

   public void setMaxTorque(double maxTorque)
   {
      this.maxTorque.setValue(maxTorque);
   }
}
