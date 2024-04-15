package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.KickDoorActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class KickDoorActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public static final double KICK_HEIGHT = 0.55;
   public static final double KICK_IMPULSE = 55.0;
   public static final double KICK_TARGET_DISTANCE = 0.75;
   public static final double PREKICK_WEIGHT_DISTRIBUTION = 0.5;
   public static final double STANCE_FOOT_WIDTH = 0.23;

   private final CRDTUnidirectionalString parentFrameName;

   private final CRDTUnidirectionalEnumField<RobotSide> kickSide;
   private final CRDTUnidirectionalDouble kickHeight;
   private final CRDTUnidirectionalDouble kickImpulse;
   private final CRDTUnidirectionalDouble kickTargetDistance;
   private final CRDTUnidirectionalDouble prekickWeightDistribution;
   private final CRDTUnidirectionalDouble stanceFootWidth;

   // On disk fields
   private String onDiskParentFrameName;
   private RobotSide onDiskSide;
   private double onDiskKickHeight;
   private double onDiskKickImpulse;
   private double onDiskKickTargetDistance;
   private double onDiskPrekickWeightDistribution;
   private double onDiskStanceFootWidth;

   public KickDoorActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());

      kickSide = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      kickHeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, KICK_HEIGHT);
      kickImpulse = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, KICK_IMPULSE);
      kickTargetDistance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, KICK_TARGET_DISTANCE);
      prekickWeightDistribution = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, PREKICK_WEIGHT_DISTRIBUTION);
      stanceFootWidth = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, STANCE_FOOT_WIDTH);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("parentFrame", parentFrameName.getValue());

      jsonNode.put("side", kickSide.getValue().getLowerCaseName());
      jsonNode.put("kickHeight", kickHeight.getValue());
      jsonNode.put("kickImpulse", kickImpulse.getValue());
      jsonNode.put("kickTargetDistance", kickTargetDistance.getValue());
      jsonNode.put("prekickWeightDistribution", prekickWeightDistribution.getValue());
      jsonNode.put("stanceFootWidth", stanceFootWidth.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());

      kickSide.setValue(RobotSide.getSideFromString(jsonNode.get("kickSide").textValue()));
      kickHeight.setValue(jsonNode.get("kickHeight").asDouble());
      kickImpulse.setValue(jsonNode.get("kickImpulse").asDouble());
      kickTargetDistance.setValue(jsonNode.get("kickTargetDistance").asDouble());
      prekickWeightDistribution.setValue(jsonNode.get("prekickWeightDistribution").asDouble());
      stanceFootWidth.setValue(jsonNode.get("stanceFootWidth").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskParentFrameName = parentFrameName.getValue();

      onDiskSide = kickSide.getValue();
      onDiskKickHeight = kickHeight.getValue();
      onDiskKickImpulse = kickImpulse.getValue();
      onDiskKickTargetDistance = kickTargetDistance.getValue();
      onDiskPrekickWeightDistribution = prekickWeightDistribution.getValue();
      onDiskStanceFootWidth = stanceFootWidth.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      parentFrameName.setValue(onDiskParentFrameName);

      kickSide.setValue(onDiskSide);
      kickHeight.setValue(onDiskKickHeight);
      kickImpulse.setValue(onDiskKickImpulse);
      kickTargetDistance.setValue(onDiskKickTargetDistance);
      prekickWeightDistribution.setValue(onDiskPrekickWeightDistribution);
      stanceFootWidth.setValue(onDiskStanceFootWidth);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);

      unchanged &= kickSide.getValue() == onDiskSide;
      unchanged &= kickHeight.getValue() == onDiskKickHeight;
      unchanged &= kickImpulse.getValue() == onDiskKickImpulse;
      unchanged &= kickTargetDistance.getValue() == onDiskKickTargetDistance;
      unchanged &= prekickWeightDistribution.getValue() == onDiskPrekickWeightDistribution;
      unchanged &= stanceFootWidth.getValue() == onDiskStanceFootWidth;

      return !unchanged;
   }

   public void toMessage(KickDoorActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(parentFrameName.toMessage());

      message.setRobotSide(kickSide.toMessage().toByte());
      message.setKickHeight(kickHeight.getValue());
      message.setKickImpulse(kickImpulse.getValue());
      message.setKickTargetDistance(kickTargetDistance.getValue());
      message.setPrekickWeightDistribution(prekickWeightDistribution.getValue());
      message.setStanceFootWidth(stanceFootWidth.getValue());
   }

   public void fromMessage(KickDoorActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      parentFrameName.fromMessage(message.getParentFrameNameAsString());

      kickSide.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      kickHeight.fromMessage(message.getKickHeight());
      kickImpulse.fromMessage(message.getKickImpulse());
      kickTargetDistance.fromMessage(message.getKickTargetDistance());
      prekickWeightDistribution.fromMessage(message.getPrekickWeightDistribution());
      stanceFootWidth.fromMessage(message.getStanceFootWidth());
   }

   @Override
   public RobotSide getSide()
   {
      return kickSide.getValue();
   }

   public void setSide(RobotSide kickSide)
   {
      this.kickSide.setValue(kickSide);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public double getKickHeight()
   {
      return kickHeight.getValue();
   }

   public void setKickHeight(double kickHeight)
   {
      this.kickHeight.setValue(kickHeight);
   }

   public double getKickImpulse()
   {
      return kickImpulse.getValue();
   }

   public void setKickImpulse(double kickImpulse)
   {
      this.kickImpulse.setValue(kickImpulse);
   }

   public double getKickTargetDistance()
   {
      return kickTargetDistance.getValue();
   }

   public void setKickTargetDistance(double kickTargetDistance)
   {
      this.kickTargetDistance.setValue(kickTargetDistance);
   }

   public double getPrekickWeightDistribution()
   {
      return prekickWeightDistribution.getValue();
   }

   public void setPrekickWeightDistribution(double prekickWeightDistribution)
   {
      this.prekickWeightDistribution.setValue(prekickWeightDistribution);
   }

   public double getStanceFootWidth()
   {
      return stanceFootWidth.getValue();
   }
}
