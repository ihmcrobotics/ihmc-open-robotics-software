package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class KickDoorApproachPlanActionDefinition extends ActionNodeDefinition
{
   public static final double KICK_IMPULSE = 55.0;
   public static final double KICK_TARGET_DISTANCE = 0.75;
   public static final double PREKICK_WEIGHT_DISTRIBUTION = 0.5;
   public static final double HORIZONTAL_DISTANCE_FROM_HANDLE = 0.1;
   public static final double STANCE_FOOT_WIDTH = 0.23;

   private final CRDTUnidirectionalDouble swingDuration;
   private final CRDTUnidirectionalDouble transferDuration;
   private final CRDTUnidirectionalEnumField<ExecutionMode> executionMode;
   private final CRDTUnidirectionalString parentFrameName;

   private final CRDTUnidirectionalEnumField<RobotSide> kickSide;
   private final CRDTUnidirectionalDouble kickImpulse;
   private final CRDTUnidirectionalDouble kickTargetDistance;
   private final CRDTUnidirectionalDouble prekickWeightDistribution;
   private final CRDTUnidirectionalDouble horizontalDistanceFromHandle;
   private final CRDTUnidirectionalDouble stanceFootWidth;

   // On disk fields
   private double onDiskSwingDuration;
   private double onDiskTransferDuration;
   private ExecutionMode onDiskExecutionMode;
   private String onDiskParentFrameName;
   private RobotSide onDiskKickSide;
   private double onDiskKickImpulse;
   private double onDiskKickTargetDistance;
   private double onDiskPrekickWeightDistribution;
   private double onDiskHorizontalDistanceFromHandle;
   private double onDiskStanceFootWidth;


   public KickDoorApproachPlanActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      swingDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.2);
      transferDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.8);
      executionMode = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, ExecutionMode.OVERRIDE);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());

      kickSide = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      kickImpulse = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, KICK_IMPULSE);
      kickTargetDistance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, KICK_TARGET_DISTANCE);
      prekickWeightDistribution = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, PREKICK_WEIGHT_DISTRIBUTION);
      horizontalDistanceFromHandle = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, HORIZONTAL_DISTANCE_FROM_HANDLE);
      stanceFootWidth = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, STANCE_FOOT_WIDTH);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("swingDuration", swingDuration.getValue());
      jsonNode.put("transferDuration", transferDuration.getValue());
      jsonNode.put("executionMode", executionMode.getValue().name());
      jsonNode.put("parentFrame", parentFrameName.getValue());

      jsonNode.put("kickImpulse", kickImpulse.getValue());
      jsonNode.put("kickTargetDistance", kickTargetDistance.getValue());
      jsonNode.put("prekickWeightDistribution", prekickWeightDistribution.getValue());
      jsonNode.put("horizontalDistanceFromHandle", horizontalDistanceFromHandle.getValue());
      jsonNode.put("stanceFootWidth", stanceFootWidth.getValue());
      jsonNode.put("kickSide", kickSide.getValue().toString());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      swingDuration.setValue(jsonNode.get("swingDuration").asDouble());
      transferDuration.setValue(jsonNode.get("transferDuration").asDouble());
      executionMode.setValue(ExecutionMode.valueOf(jsonNode.get("executionMode").textValue()));
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());

      kickImpulse.setValue(jsonNode.get("kickImpulse").asDouble());
      kickTargetDistance.setValue(jsonNode.get("kickTargetDistance").asDouble());
      prekickWeightDistribution.setValue(jsonNode.get("prekickWeightDistribution").asDouble());
      horizontalDistanceFromHandle.setValue(jsonNode.get("horizontalDistanceFromHandle").asDouble());
      stanceFootWidth.setValue(jsonNode.get("stanceFootWidth").asDouble());
      kickSide.setValue(RobotSide.getSideFromString(jsonNode.get("kickSide").textValue()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSwingDuration = swingDuration.getValue();
      onDiskTransferDuration = transferDuration.getValue();
      onDiskExecutionMode = executionMode.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskKickSide = kickSide.getValue();
      onDiskKickImpulse = kickImpulse.getValue();
      onDiskKickTargetDistance = kickTargetDistance.getValue();
      onDiskPrekickWeightDistribution = prekickWeightDistribution.getValue();
      onDiskHorizontalDistanceFromHandle = horizontalDistanceFromHandle.getValue();
      onDiskStanceFootWidth = stanceFootWidth.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      swingDuration.setValue(onDiskSwingDuration);
      transferDuration.setValue(onDiskTransferDuration);
      executionMode.setValue(onDiskExecutionMode);
      parentFrameName.setValue(onDiskParentFrameName);
      kickSide.setValue(onDiskKickSide);
      kickImpulse.setValue(onDiskKickImpulse);
      kickTargetDistance.setValue(onDiskKickTargetDistance);
      prekickWeightDistribution.setValue(onDiskPrekickWeightDistribution);
      horizontalDistanceFromHandle.setValue(onDiskHorizontalDistanceFromHandle);
      stanceFootWidth.setValue(onDiskStanceFootWidth);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= swingDuration.getValue() == onDiskSwingDuration;
      unchanged &= transferDuration.getValue() == onDiskTransferDuration;
      unchanged &= executionMode.getValue() == onDiskExecutionMode;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= kickSide.getValue() == onDiskKickSide;
      unchanged &= kickImpulse.getValue() == onDiskKickImpulse;
      unchanged &= kickTargetDistance.getValue() == onDiskKickTargetDistance;
      unchanged &= prekickWeightDistribution.getValue() == onDiskPrekickWeightDistribution;
      unchanged &= horizontalDistanceFromHandle.getValue() == onDiskHorizontalDistanceFromHandle;
      unchanged &= stanceFootWidth.getValue() == onDiskStanceFootWidth;

      return !unchanged;
   }

   public void toMessage(KickDoorApproachPlanDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setExecutionMode(executionMode.toMessageOrdinal());
      message.setParentFrameName(parentFrameName.toMessage());

      message.setRobotSide(kickSide.toMessage().toByte());
      message.setKickImpulse(kickImpulse.toMessage());
      message.setKickTargetDistance(kickTargetDistance.toMessage());
      message.setPrekickWeightDistribution(prekickWeightDistribution.toMessage());
      message.setHorizontalDistanceFromHandle(horizontalDistanceFromHandle.toMessage());
      message.setStanceFootWidth(stanceFootWidth.toMessage());
   }

   public void fromMessage(KickDoorApproachPlanDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      executionMode.fromMessageOrdinal(message.getExecutionMode(), ExecutionMode.values);
      parentFrameName.fromMessage(message.getParentFrameNameAsString());

      kickSide.setValue(RobotSide.fromByte(message.getRobotSide()));
      kickImpulse.setValue(message.getKickImpulse());
      kickTargetDistance.setValue(message.getKickTargetDistance());
      prekickWeightDistribution.setValue(message.getPrekickWeightDistribution());
      horizontalDistanceFromHandle.setValue(message.getHorizontalDistanceFromHandle());
      stanceFootWidth.setValue(message.getStanceFootWidth());
   }

   public double getSwingDuration()
   {
      return swingDuration.getValue();
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.setValue(swingDuration);
   }

   public double getTransferDuration()
   {
      return transferDuration.getValue();
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration.setValue(transferDuration);
   }

   public CRDTUnidirectionalEnumField<ExecutionMode> getExecutionMode()
   {
      return executionMode;
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTUnidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTUnidirectionalEnumField<RobotSide> getKickSide()
   {
      return kickSide;
   }

   public CRDTUnidirectionalDouble getKickImpulse()
   {
      return kickImpulse;
   }

   public CRDTUnidirectionalDouble getKickTargetDistance()
   {
      return kickTargetDistance;
   }

   public CRDTUnidirectionalDouble getPrekickWeightDistribution()
   {
      return prekickWeightDistribution;
   }

   public CRDTUnidirectionalDouble getHorizontalDistanceFromHandle()
   {
      return horizontalDistanceFromHandle;
   }

   public CRDTUnidirectionalDouble getStanceFootWidth()
   {
      return stanceFootWidth;
   }
}
