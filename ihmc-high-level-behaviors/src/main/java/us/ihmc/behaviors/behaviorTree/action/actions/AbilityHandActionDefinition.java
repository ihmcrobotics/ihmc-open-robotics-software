package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalFloatArray;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;

public class AbilityHandActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public enum SuccessCriteria
   {
      CHECK_EACH_JOINT_POSITION,
      WAIT_ONLY,
      CHECK_CUMULATIVE_JOINT_MOVEMENT;

      public static final SuccessCriteria[] values = values();
   }

   private final CRDTBidirectionalEnumField<RobotSide> side;
   private final CRDTBidirectionalEnumField<ControlMode> controlMode;
   private final CRDTBidirectionalEnumField<Grip> grip;
   private final CRDTBidirectionalFloatArray goalPositions;
   private final CRDTBidirectionalFloatArray goalVelocities;
   private final CRDTBidirectionalEnumField<SuccessCriteria> successCriteria;
   private final CRDTBidirectionalFloat eachJointPositionTolerance;
   private final CRDTBidirectionalFloat sufficientCumulativeJointMovement;
   private final CRDTBidirectionalBoolean enableWiggleOnFailure;
   private final CRDTBidirectionalFloat timeToWiggle;
   private final CRDTBidirectionalFloat ultimateTimeout;

   // On disk fields
   private RobotSide onDiskSide;
   private ControlMode onDiskControlMode;
   private Grip onDiskGrip;
   private final float[] onDiskGoalPositions = new float[6];
   private final float[] onDiskGoalVelocities = new float[6];
   private SuccessCriteria onDiskSuccessCriteria;
   private float onDiskEachJointPositionTolerance;
   private float onDiskSufficientCumulativeJointMovement;
   private boolean onDiskEnableWiggleOnFailure;
   private float onDiskTimeToWiggle;
   private float onDiskUltimateTimeout;

   public AbilityHandActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      side = new CRDTBidirectionalEnumField<>(this, RobotSide.LEFT);
      controlMode = new CRDTBidirectionalEnumField<>(this, ControlMode.GRIP);
      grip = new CRDTBidirectionalEnumField<>(this, Grip.RELAX);
      goalPositions = new CRDTBidirectionalFloatArray(this, 6);
      goalVelocities = new CRDTBidirectionalFloatArray(this, 6);
      for (int i = 0; i < 6; i++)
         goalVelocities.getValue()[i] = 30.0f; // important not to modify to set initial values
      successCriteria = new CRDTBidirectionalEnumField<>(this, SuccessCriteria.CHECK_EACH_JOINT_POSITION);
      eachJointPositionTolerance = new CRDTBidirectionalFloat(this, 10.0f);
      sufficientCumulativeJointMovement = new CRDTBidirectionalFloat(this, 50.0f);
      enableWiggleOnFailure = new CRDTBidirectionalBoolean(this, false);
      timeToWiggle = new CRDTBidirectionalFloat(this, 5.0f);
      ultimateTimeout = new CRDTBidirectionalFloat(this, 10.0f);
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
      jsonNode.put("successCriteria", successCriteria.getValue().name());
      switch (successCriteria.getValue())
      {
         case CHECK_EACH_JOINT_POSITION -> jsonNode.put("eachJointPositionTolerance", eachJointPositionTolerance.getValue());
         case CHECK_CUMULATIVE_JOINT_MOVEMENT -> jsonNode.put("sufficientCumulativeJointMovement", sufficientCumulativeJointMovement.getValue());
      }
      jsonNode.put("enableWiggleOnFailure", enableWiggleOnFailure.getValue());
      if (enableWiggleOnFailure.getValue())
         jsonNode.put("timeToWiggle", timeToWiggle.getValue());
      jsonNode.put("ultimateTimeout", ultimateTimeout.getValue());
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
      successCriteria.setValue(SuccessCriteria.valueOf(jsonNode.get("successCriteria").asText()));
      switch (successCriteria.getValue())
      {
         case CHECK_EACH_JOINT_POSITION -> eachJointPositionTolerance.setValue((float) jsonNode.get("eachJointPositionTolerance").asDouble());
         case CHECK_CUMULATIVE_JOINT_MOVEMENT -> sufficientCumulativeJointMovement.setValue((float) jsonNode.get("sufficientCumulativeJointMovement").asDouble());
      }
      enableWiggleOnFailure.setValue(jsonNode.get("enableWiggleOnFailure").asBoolean());
      if (enableWiggleOnFailure.getValue())
         timeToWiggle.setValue((float) jsonNode.get("timeToWiggle").asDouble());
      ultimateTimeout.setValue((float) jsonNode.get("ultimateTimeout").asDouble());
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
      onDiskSuccessCriteria = successCriteria.getValue();
      onDiskEachJointPositionTolerance = eachJointPositionTolerance.getValue();
      onDiskSufficientCumulativeJointMovement = sufficientCumulativeJointMovement.getValue();
      onDiskEnableWiggleOnFailure = enableWiggleOnFailure.getValue();
      onDiskTimeToWiggle = timeToWiggle.getValue();
      onDiskUltimateTimeout = ultimateTimeout.getValue();
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
         successCriteria.setValue(onDiskSuccessCriteria);
         eachJointPositionTolerance.setValue(onDiskEachJointPositionTolerance);
         sufficientCumulativeJointMovement.setValue(onDiskSufficientCumulativeJointMovement);
         enableWiggleOnFailure.setValue(onDiskEnableWiggleOnFailure);
         timeToWiggle.setValue(onDiskTimeToWiggle);
         ultimateTimeout.setValue(onDiskUltimateTimeout);
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
      unchanged &= successCriteria.getValue() == onDiskSuccessCriteria;
      unchanged &= eachJointPositionTolerance.getValue() == onDiskEachJointPositionTolerance;
      unchanged &= sufficientCumulativeJointMovement.getValue() == onDiskSufficientCumulativeJointMovement;
      unchanged &= enableWiggleOnFailure.getValue() == onDiskEnableWiggleOnFailure;
      unchanged &= timeToWiggle.getValue() == onDiskTimeToWiggle;
      unchanged &= ultimateTimeout.getValue() == onDiskUltimateTimeout;

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
      message.setSuccessCriteria(successCriteria.toMessageOrdinal());
      message.setEachJointPositionTolerance(eachJointPositionTolerance.toMessage());
      message.setSufficientCumulativeJointMovement(sufficientCumulativeJointMovement.toMessage());
      message.setEnableWiggleOnFailure(enableWiggleOnFailure.toMessage());
      message.setTimeToWiggle(timeToWiggle.toMessage());
      message.setUltimateTimeout(ultimateTimeout.toMessage());
   }

   public void fromMessage(AbilityHandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      controlMode.fromMessageOrdinal(message.getControlMode(), ControlMode.values);
      grip.fromMessageOrdinal(message.getGrip(), Grip.values);
      goalPositions.fromMessage(message.getGoalPositions());
      goalVelocities.fromMessage(message.getGoalVelocities());
      successCriteria.fromMessageOrdinal(message.getSuccessCriteria(), SuccessCriteria.values);
      eachJointPositionTolerance.fromMessage(message.getEachJointPositionTolerance());
      sufficientCumulativeJointMovement.fromMessage(message.getSufficientCumulativeJointMovement());
      enableWiggleOnFailure.fromMessage(message.getEnableWiggleOnFailure());
      timeToWiggle.fromMessage(message.getTimeToWiggle());
      ultimateTimeout.fromMessage(message.getUltimateTimeout());
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

   public SuccessCriteria getSuccessCriteria()
   {
      return successCriteria.getValue();
   }

   public void setSuccessCriteria(SuccessCriteria successCriteria)
   {
      this.successCriteria.setValue(successCriteria);
   }

   public float getEachJointPositionTolerance()
   {
      return eachJointPositionTolerance.getValue();
   }

   public void setEachJointPositionTolerance(float eachJointPositionTolerance)
   {
      this.eachJointPositionTolerance.setValue(eachJointPositionTolerance);
   }

   public float getSufficientCumulativeJointMovement()
   {
      return sufficientCumulativeJointMovement.getValue();
   }

   public void setSufficientCumulativeJointMovement(float sufficientCumulativeJointMovement)
   {
      this.sufficientCumulativeJointMovement.setValue(sufficientCumulativeJointMovement);
   }

   public boolean getEnableWiggleOnFailure()
   {
      return enableWiggleOnFailure.getValue();
   }

   public void setEnableWiggleOnFailure(boolean enableWiggleOnFailure)
   {
      this.enableWiggleOnFailure.setValue(enableWiggleOnFailure);
   }

   public float getTimeToWiggle()
   {
      return timeToWiggle.getValue();
   }

   public void setTimeToWiggle(float timeToWiggle)
   {
      this.timeToWiggle.setValue(timeToWiggle);
   }

   public float getUltimateTimeout()
   {
      return ultimateTimeout.getValue();
   }

   public void setUltimateTimeout(float ultimateTimeout)
   {
      this.ultimateTimeout.setValue(ultimateTimeout);
   }
}
