package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class SakeHandCommandActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalInteger handConfigurationIndex;
   private final CRDTUnidirectionalDouble goalPosition;
   private final CRDTUnidirectionalDouble goalTorque;

   public SakeHandCommandActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      handConfigurationIndex = new CRDTUnidirectionalInteger(ROS2ActorDesignation.OPERATOR, crdtInfo, SakeHandCommandOption.GOTO.ordinal());
      goalPosition = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0); // default to open
      goalTorque = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0); // default to none
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("configuration", SakeHandCommandOption.values[handConfigurationIndex.getValue()].name());
      jsonNode.put("position", goalPosition.getValue());
      jsonNode.put("torque", goalTorque.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      handConfigurationIndex.setValue(SakeHandCommandOption.valueOf(jsonNode.get("configuration").asText()).ordinal());
      goalPosition.setValue(jsonNode.get("position").asDouble());
      goalTorque.setValue(jsonNode.get("torque").asDouble());
   }

   public void toMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setConfiguration(handConfigurationIndex.toMessage());
      message.setPositionRatio(goalPosition.toMessage());
      message.setTorqueRatio(goalTorque.toMessage());
   }

   public void fromMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      handConfigurationIndex.fromMessage((int) message.getConfiguration());
      goalPosition.fromMessage(message.getPositionRatio());
      goalTorque.fromMessage(message.getTorqueRatio());
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

   public SakeHandCommandOption getSakeCommandOption()
   {
      return SakeHandCommandOption.values[handConfigurationIndex.getValue()];
   }

   public void setHandConfigurationIndex(int handConfigurationIndex)
   {
      this.handConfigurationIndex.setValue(handConfigurationIndex);
   }

   public double getGoalPosition()
   {
      return goalPosition.getValue();
   }

   public double getGoalTorque()
   {
      return goalTorque.getValue();
   }

   public void setGoalPosition(double goalPosition)
   {
      this.goalPosition.setValue(goalPosition);
   }

   public void setGoalTorque(double goalTorque)
   {
      this.goalTorque.setValue(goalTorque);
   }
}
