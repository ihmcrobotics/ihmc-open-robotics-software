package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class SakeHandCommandActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble handOpenAngle;
   private final CRDTUnidirectionalDouble fingertipGripForceLimit;

   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskHandOpenAngle;
   private double onDiskFingertipGripForceLimit;

   public SakeHandCommandActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      handOpenAngle = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, SakeHandPreset.OPEN.getHandOpenAngle());
      fingertipGripForceLimit = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("handOpenAngleDegrees", Double.parseDouble("%.1f".formatted(Math.toDegrees(handOpenAngle.getValue()))));
      jsonNode.put("fingertipGripForceLimit", Double.parseDouble("%.1f".formatted(fingertipGripForceLimit.getValue())));
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      handOpenAngle.setValue(Math.toRadians(jsonNode.get("handOpenAngleDegrees").asDouble()));
      fingertipGripForceLimit.setValue(jsonNode.get("fingertipGripForceLimit").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskHandOpenAngle = handOpenAngle.getValue();
      onDiskFingertipGripForceLimit = fingertipGripForceLimit.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      handOpenAngle.setValue(onDiskHandOpenAngle);
      fingertipGripForceLimit.setValue(onDiskFingertipGripForceLimit);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= handOpenAngle.getValue() == onDiskHandOpenAngle;
      unchanged &= fingertipGripForceLimit.getValue() == onDiskFingertipGripForceLimit;

      return !unchanged;
   }

   public void toMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setHandOpenAngle(handOpenAngle.toMessage());
      message.setFingertipGripForceLimit(fingertipGripForceLimit.toMessage());
   }

   public void fromMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      handOpenAngle.fromMessage(message.getHandOpenAngle());
      fingertipGripForceLimit.fromMessage(message.getFingertipGripForceLimit());
   }

   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public double getHandOpenAngle()
   {
      return handOpenAngle.getValue();
   }

   public double getFingertipGripForceLimit()
   {
      return fingertipGripForceLimit.getValue();
   }

   public void setHandOpenAngle(double handOpenAngle)
   {
      this.handOpenAngle.setValue(handOpenAngle);
   }

   public void setFingertipGripForceLimit(double fingertipGripForceLimit)
   {
      this.fingertipGripForceLimit.setValue(fingertipGripForceLimit);
   }
}
