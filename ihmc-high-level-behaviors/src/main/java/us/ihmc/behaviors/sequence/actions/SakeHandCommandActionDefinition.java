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
   /** If it's already within this amount to the goal, we will skip and mark as success. */
   public static final double DEFAULT_HAND_ANGLE_INITIAL_SATISFACTION_TOLERANCE = Math.toRadians(5.0);
   /** We usually want to allow a bunch of compliance. */
   public static final double DEFAULT_HAND_ANGLE_COMPLETION_TOLERANCE = Math.toRadians(40.0);

   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble handOpenAngle;
   private final CRDTUnidirectionalDouble initialSatisfactionHandAngleTolerance;
   private final CRDTUnidirectionalDouble completionHandAngleTolerance;
   private final CRDTUnidirectionalDouble fingertipGripForceLimit;

   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskHandOpenAngle;
   private double onDiskInitialSatisfactionHandAngleTolerance;
   private double onDiskCompletionHandAngleTolerance;
   private double onDiskFingertipGripForceLimit;

   public SakeHandCommandActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      handOpenAngle = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, SakeHandPreset.OPEN.getHandOpenAngle());
      initialSatisfactionHandAngleTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR,
                                                                           crdtInfo,
                                                                           DEFAULT_HAND_ANGLE_INITIAL_SATISFACTION_TOLERANCE);
      completionHandAngleTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_HAND_ANGLE_COMPLETION_TOLERANCE);
      fingertipGripForceLimit = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("handOpenAngleDegrees", Double.parseDouble("%.1f".formatted(Math.toDegrees(handOpenAngle.getValue()))));
      jsonNode.put("initialSatisfactionHandAngleTolerance",
                   Double.parseDouble("%.1f".formatted(Math.toDegrees(initialSatisfactionHandAngleTolerance.getValue()))));
      jsonNode.put("completionHandAngleTolerance", Double.parseDouble("%.1f".formatted(Math.toDegrees(completionHandAngleTolerance.getValue()))));
      jsonNode.put("fingertipGripForceLimit", Double.parseDouble("%.1f".formatted(fingertipGripForceLimit.getValue())));
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      handOpenAngle.setValue(Math.toRadians(jsonNode.get("handOpenAngleDegrees").asDouble()));
      initialSatisfactionHandAngleTolerance.setValue(Math.toRadians(jsonNode.get("initialSatisfactionHandAngleTolerance").asDouble()));
      completionHandAngleTolerance.setValue(Math.toRadians(jsonNode.get("completionHandAngleTolerance").asDouble()));
      fingertipGripForceLimit.setValue(jsonNode.get("fingertipGripForceLimit").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskHandOpenAngle = handOpenAngle.getValue();
      onDiskInitialSatisfactionHandAngleTolerance = initialSatisfactionHandAngleTolerance.getValue();
      onDiskCompletionHandAngleTolerance = completionHandAngleTolerance.getValue();
      onDiskFingertipGripForceLimit = fingertipGripForceLimit.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      handOpenAngle.setValue(onDiskHandOpenAngle);
      initialSatisfactionHandAngleTolerance.setValue(onDiskInitialSatisfactionHandAngleTolerance);
      completionHandAngleTolerance.setValue(onDiskCompletionHandAngleTolerance);
      fingertipGripForceLimit.setValue(onDiskFingertipGripForceLimit);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= handOpenAngle.getValue() == onDiskHandOpenAngle;
      unchanged &= initialSatisfactionHandAngleTolerance.getValue() == onDiskInitialSatisfactionHandAngleTolerance;
      unchanged &= completionHandAngleTolerance.getValue() == onDiskCompletionHandAngleTolerance;
      unchanged &= fingertipGripForceLimit.getValue() == onDiskFingertipGripForceLimit;

      return !unchanged;
   }

   public void toMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setHandOpenAngle(handOpenAngle.toMessage());
      message.setInitialSatisfactionHandAngleTolerance(initialSatisfactionHandAngleTolerance.toMessage());
      message.setCompletionHandAngleTolerance(completionHandAngleTolerance.toMessage());
      message.setFingertipGripForceLimit(fingertipGripForceLimit.toMessage());
   }

   public void fromMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      handOpenAngle.fromMessage(message.getHandOpenAngle());
      initialSatisfactionHandAngleTolerance.fromMessage(message.getInitialSatisfactionHandAngleTolerance());
      completionHandAngleTolerance.fromMessage(message.getCompletionHandAngleTolerance());
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

   public void setHandOpenAngle(double handOpenAngle)
   {
      this.handOpenAngle.setValue(handOpenAngle);
   }

   public double getInitialSatisfactionHandAngleTolerance()
   {
      return initialSatisfactionHandAngleTolerance.getValue();
   }

   public void setInitialSatisfactionHandAngleTolerance(double initialSatisfactionHandAngleTolerance)
   {
      this.initialSatisfactionHandAngleTolerance.setValue(initialSatisfactionHandAngleTolerance);
   }

   public double getCompletionHandAngleTolerance()
   {
      return completionHandAngleTolerance.getValue();
   }

   public void setCompletionHandAngleTolerance(double completionHandAngleTolerance)
   {
      this.completionHandAngleTolerance.setValue(completionHandAngleTolerance);
   }

   public double getFingertipGripForceLimit()
   {
      return fingertipGripForceLimit.getValue();
   }

   public void setFingertipGripForceLimit(double fingertipGripForceLimit)
   {
      this.fingertipGripForceLimit.setValue(fingertipGripForceLimit);
   }
}
