package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandWrenchActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public static final double DEFAULT_FORCE = 5.0;
   public static final double DEFAULT_TORQUE = 5.0;

   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalDouble forceX;
   private final CRDTUnidirectionalDouble forceY;
   private final CRDTUnidirectionalDouble forceZ;
   private final CRDTUnidirectionalDouble torqueX;
   private final CRDTUnidirectionalDouble torqueY;
   private final CRDTUnidirectionalDouble torqueZ;


   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskTrajectoryDuration;
   private double onDiskForce;

   public HandWrenchActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
      forceX = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_FORCE);
      forceY = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_FORCE);
      forceZ = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_FORCE);
      torqueX = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_TORQUE);
      torqueY = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_TORQUE);
      torqueZ = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_TORQUE);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("forceX", forceX.getValue());
      jsonNode.put("forceY", forceY.getValue());
      jsonNode.put("forceZ", forceZ.getValue());
      jsonNode.put("torqueX", torqueX.getValue());
      jsonNode.put("torqueY", torqueY.getValue());
      jsonNode.put("torqueZ", torqueZ.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      forceX.setValue(jsonNode.get("forceX").asDouble());
      forceY.setValue(jsonNode.get("forceY").asDouble());
      forceZ.setValue(jsonNode.get("forceZ").asDouble());
      torqueX.setValue(jsonNode.get("torqueX").asDouble());
      torqueY.setValue(jsonNode.get("torqueY").asDouble());
      torqueZ.setValue(jsonNode.get("torqueZ").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskForce = force.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      force.setValue(onDiskForce);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= force.getValue() == onDiskForce;

      return !unchanged;
   }

   public void toMessage(HandWrenchActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setForceX(forceX.toMessage());
      message.setForceY(forceY.toMessage());
      message.setForceZ(forceZ.toMessage());
      message.setTorqueX(torqueX.toMessage());
      message.setTorqueY(torqueY.toMessage());
      message.setTorqueZ(torqueZ.toMessage());
   }

   public void fromMessage(HandWrenchActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      forceX.fromMessage(message.getForceX());
      forceY.fromMessage(message.getForceY());
      forceZ.fromMessage(message.getForceZ());
      torqueX.fromMessage(message.getTorqueX());
      torqueY.fromMessage(message.getTorqueY());
      torqueZ.fromMessage(message.getTorqueZ());
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }
   
   @Override
   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public double getForceX()
   {
      return forceX.getValue();
   }

   public void setForceX(double force)
   {
      this.forceX.setValue(force);
   }

   public double getForceY()
   {
      return forceY.getValue();
   }

   public void setForceY(double force)
   {
      this.forceY.setValue(force);
   }

   public double getForceZ()
   {
      return forceZ.getValue();
   }

   public void setForceZ(double force)
   {
      this.forceZ.setValue(force);
   }

   public double getTorqueX()
   {
      return torqueX.getValue();
   }

   public void setTorqueX(double force)
   {
      this.torqueX.setValue(force);
   }

   public double getTorqueY()
   {
      return torqueY.getValue();
   }

   public void setTorqueY(double force)
   {
      this.torqueY.setValue(force);
   }

   public double getTorqueZ()
   {
      return torqueZ.getValue();
   }

   public void setTorqueZ(double force)
   {
      this.torqueZ.setValue(force);
   }
}
