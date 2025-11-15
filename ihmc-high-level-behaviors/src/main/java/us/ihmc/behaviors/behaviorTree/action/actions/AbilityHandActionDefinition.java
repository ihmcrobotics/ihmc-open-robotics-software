package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalFloatArray;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;

public class AbilityHandActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTBidirectionalEnumField<RobotSide> side;
   private final CRDTBidirectionalEnumField<ControlMode> controlMode;
   private final CRDTBidirectionalEnumField<Grip> grip;
   private final CRDTBidirectionalFloatArray goalPositions;
   private final CRDTBidirectionalFloatArray goalVelocities;

   // On disk fields
   private RobotSide onDiskSide;
   private ControlMode onDiskControlMode;
   private Grip onDiskGrip;
   private final float[] onDiskGoalPositions = new float[6];
   private final float[] onDiskGoalVelocities = new float[6];

   public AbilityHandActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      side = new CRDTBidirectionalEnumField<>(this, RobotSide.LEFT);
      controlMode = new CRDTBidirectionalEnumField<>(this, ControlMode.GRIP);
      grip = new CRDTBidirectionalEnumField<>(this, Grip.PINCH_O);
      goalPositions = new CRDTBidirectionalFloatArray(this, 6);
      goalVelocities = new CRDTBidirectionalFloatArray(this, 6);
      for (int i = 0; i < 6; i++)
         goalVelocities.getValue()[i] = 30.0f; // important not to modify to set initial values
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("controlMode", controlMode.getValue().name());
      jsonNode.put("grip", grip.getValue().name());
      jsonNode.putPOJO("goalPositions", goalPositions.getValue());
      jsonNode.putPOJO("goalVelocities", goalVelocities.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      controlMode.setValue(ControlMode.valueOf(jsonNode.get("controlMode").asText()));
      grip.setValue(Grip.valueOf(jsonNode.get("grip").asText()));
      for (int i = 0; i < 6; i++)
         goalPositions.setValue(i, (float) jsonNode.get("goalPositions").get(i).asDouble());
      for (int i = 0; i < 6; i++)
         goalVelocities.setValue(i, (float) jsonNode.get("goalVelocities").get(i).asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskControlMode = controlMode.getValue();
      onDiskGrip = grip.getValue();
      goalPositions.getValue(onDiskGoalPositions);
      goalVelocities.getValue(onDiskGoalVelocities);
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         side.setValue(onDiskSide);
         controlMode.setValue(onDiskControlMode);
         grip.setValue(onDiskGrip);
         goalPositions.setValue(onDiskGoalPositions);
         goalVelocities.setValue(onDiskGoalVelocities);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= controlMode.getValue() == onDiskControlMode;
      unchanged &= grip.getValue() == onDiskGrip;
      for (int i = 0; i < 6; i++)
      {
         unchanged &= goalPositions.getValue()[i] == onDiskGoalPositions[i];
         unchanged &= goalVelocities.getValue()[i] == onDiskGoalVelocities[i];
      }

      return !unchanged;
   }

   public void toMessage(AbilityHandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setControlMode(controlMode.toMessageOrdinal());
      message.setGrip(grip.toMessageOrdinal());
      goalPositions.toMessage(message.getGoalPositions());
      goalVelocities.toMessage(message.getGoalVelocities());
   }

   public void fromMessage(AbilityHandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      controlMode.fromMessageOrdinal(message.getControlMode(), ControlMode.values);
      grip.fromMessageOrdinal(message.getGrip(), Grip.values);
      goalPositions.fromMessage(message.getGoalPositions());
      goalVelocities.fromMessage(message.getGoalVelocities());
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

   public ControlMode getControlMode()
   {
      return controlMode.getValue();
   }

   public void setControlMode(ControlMode controlMode)
   {
      this.controlMode.setValue(controlMode);
   }

   public Grip getGrip()
   {
      return grip.getValue();
   }

   public void setGrip(Grip grip)
   {
      this.grip.setValue(grip);
   }

   public CRDTBidirectionalFloatArray getGoalPositions()
   {
      return goalPositions;
   }

   public CRDTBidirectionalFloatArray getGoalVelocities()
   {
      return goalVelocities;
   }
}
