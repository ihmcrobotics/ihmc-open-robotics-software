package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.msg.dds.AI2RNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class AI2RNodeDefinition extends BehaviorTreeNodeDefinition
{
   public static final int DEFAULT_NUMBER_OF_RANDOMIZATIONS = 1;

   private final CRDTBidirectionalBoolean randomizeGoToAction;
   private final CRDTBidirectionalBoolean randomizeWholeBodyAction;
   private final CRDTBidirectionalInteger numberOfRandomizations;
   private final CRDTBidirectionalInteger wholeBodyRandomizationRequestID;
   private final CRDTBidirectionalRigidBodyTransform rightHandTransformToChest;
   private final CRDTBidirectionalRigidBodyTransform leftHandTransformToChest;
   private final CRDTBidirectionalRigidBodyTransform pelvisTransformToParent;
   private final CRDTBidirectionalDouble spinePitchDegrees;
   private final CRDTBidirectionalDouble spineRollDegrees;
   private final CRDTBidirectionalDouble spineYawDegrees;
   private final CRDTBidirectionalDouble rightHandTrajectoryDuration;
   private final CRDTBidirectionalDouble leftHandTrajectoryDuration;
   private final CRDTBidirectionalDouble spineTrajectoryDuration;
   private final CRDTBidirectionalDouble pelvisTrajectoryDuration;
   private final CRDTBidirectionalInteger leftHandMask;
   private final CRDTBidirectionalInteger rightHandMask;
   private final CRDTBidirectionalInteger spineMask;
   private final CRDTBidirectionalInteger pelvisMask;
   private final CRDTBidirectionalDouble probabilityRightArmEnabled;
   private final CRDTBidirectionalDouble probabilityLeftArmEnabled;
   private final CRDTBidirectionalDouble probabilityTorsoEnabled;
   private final CRDTBidirectionalDouble probabilityPelvisEnabled;

   private boolean onDiskRandomizeGoToAction;
   private boolean onDiskRandomizeWholeBodyAction;
   private int onDiskNumberOfRandomizations;
   private int onDiskWholeBodyRandomizationRequestID;
   private final RigidBodyTransform onDiskRightHandTransformToChest = new RigidBodyTransform();
   private final RigidBodyTransform onDiskLeftHandTransformToChest = new RigidBodyTransform();
   private final RigidBodyTransform onDiskPelvisTransformToParent = new RigidBodyTransform();
   private double onDiskSpinePitchDegrees;
   private double onDiskSpineRollDegrees;
   private double onDiskSpineYawDegrees;
   private double onDiskRightHandTrajectoryDuration;
   private double onDiskLeftHandTrajectoryDuration;
   private double onDiskSpineTrajectoryDuration;
   private double onDiskPelvisTrajectoryDuration;
   private int onDiskLeftHandMask;
   private int onDiskRightHandMask;
   private int onDiskSpineMask;
   private int onDiskPelvisMask;
   private double onDiskProbabilityRightArmEnabled;
   private double onDiskProbabilityLeftArmEnabled;
   private double onDiskProbabilityTorsoEnabled;
   private double onDiskProbabilityPelvisEnabled;

   public AI2RNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      randomizeGoToAction = new CRDTBidirectionalBoolean(this, false);
      randomizeWholeBodyAction = new CRDTBidirectionalBoolean(this, false);
      numberOfRandomizations = new CRDTBidirectionalInteger(this, DEFAULT_NUMBER_OF_RANDOMIZATIONS);
      wholeBodyRandomizationRequestID = new CRDTBidirectionalInteger(this, 0);
      rightHandTransformToChest = new CRDTBidirectionalRigidBodyTransform(this);
      rightHandTransformToChest.getValueAndModify().getTranslation().set(0.163, -0.554, 0.137);
      rightHandTransformToChest.getValueAndModify().getRotation().setYawPitchRoll(Math.toRadians(36.46), Math.toRadians(-65.68), Math.toRadians(-31.52));
      leftHandTransformToChest = new CRDTBidirectionalRigidBodyTransform(this);
      leftHandTransformToChest.getValueAndModify().getTranslation().set(0.134, 0.327, 0.0865);
      leftHandTransformToChest.getValueAndModify().getRotation().setYawPitchRoll(Math.toRadians(-36.46), Math.toRadians(-65.68), Math.toRadians(31.52));
      pelvisTransformToParent = new CRDTBidirectionalRigidBodyTransform(this);
      pelvisTransformToParent.getValueAndModify().getTranslation().set(0.0, 0.0, 0.6645);
      spinePitchDegrees = new CRDTBidirectionalDouble(this, -15.0);
      spineRollDegrees = new CRDTBidirectionalDouble(this, 0.0);
      spineYawDegrees = new CRDTBidirectionalDouble(this, 0.0);
      rightHandTrajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      leftHandTrajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      spineTrajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      pelvisTrajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      leftHandMask = new CRDTBidirectionalInteger(this, 1);
      rightHandMask = new CRDTBidirectionalInteger(this, 1);
      spineMask = new CRDTBidirectionalInteger(this, 1);
      pelvisMask = new CRDTBidirectionalInteger(this, 1);
      probabilityRightArmEnabled = new CRDTBidirectionalDouble(this, 0.5);
      probabilityLeftArmEnabled = new CRDTBidirectionalDouble(this, 0.5);
      probabilityTorsoEnabled = new CRDTBidirectionalDouble(this, 0.2);
      probabilityPelvisEnabled = new CRDTBidirectionalDouble(this, 0.2);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);
      jsonNode.put("randomizeGoToAction", randomizeGoToAction.getValue());
      jsonNode.put("randomizeWholeBodyAction", randomizeWholeBodyAction.getValue());
      jsonNode.put("numberOfRandomizations", numberOfRandomizations.getValue());
      jsonNode.put("wholeBodyRandomizationRequestID", wholeBodyRandomizationRequestID.getValue());
      JSONTools.toJSON(jsonNode.putObject("rightHandTransformToChest"), rightHandTransformToChest.getValueReadOnly());
      JSONTools.toJSON(jsonNode.putObject("leftHandTransformToChest"), leftHandTransformToChest.getValueReadOnly());
      JSONTools.toJSON(jsonNode.putObject("pelvisTransformToParent"), pelvisTransformToParent.getValueReadOnly());
      jsonNode.put("spinePitchDegrees", spinePitchDegrees.getValue());
      jsonNode.put("spineRollDegrees", spineRollDegrees.getValue());
      jsonNode.put("spineYawDegrees", spineYawDegrees.getValue());
      jsonNode.put("rightHandTrajectoryDuration", rightHandTrajectoryDuration.getValue());
      jsonNode.put("leftHandTrajectoryDuration", leftHandTrajectoryDuration.getValue());
      jsonNode.put("spineTrajectoryDuration", spineTrajectoryDuration.getValue());
      jsonNode.put("pelvisTrajectoryDuration", pelvisTrajectoryDuration.getValue());
      jsonNode.put("leftHandMask", leftHandMask.getValue());
      jsonNode.put("rightHandMask", rightHandMask.getValue());
      jsonNode.put("spineMask", spineMask.getValue());
      jsonNode.put("pelvisMask", pelvisMask.getValue());
      jsonNode.put("probabilityRightArmEnabled", probabilityRightArmEnabled.getValue());
      jsonNode.put("probabilityLeftArmEnabled", probabilityLeftArmEnabled.getValue());
      jsonNode.put("probabilityTorsoEnabled", probabilityTorsoEnabled.getValue());
      jsonNode.put("probabilityPelvisEnabled", probabilityPelvisEnabled.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);
      if (jsonNode.has("randomizeGoToAction"))
         setRandomizeGoToActionEnabled(jsonNode.get("randomizeGoToAction").asBoolean());
      if (jsonNode.has("randomizeWholeBodyAction"))
         setRandomizeWholeBodyActionEnabled(jsonNode.get("randomizeWholeBodyAction").asBoolean());
      if (jsonNode.has("numberOfRandomizations"))
         setNumberOfRandomizationsValue(jsonNode.get("numberOfRandomizations").asInt());
      if (jsonNode.has("wholeBodyRandomizationRequestID"))
         setWholeBodyRandomizationRequestID(jsonNode.get("wholeBodyRandomizationRequestID").asInt());
      if (jsonNode.get("rightHandTransformToChest") instanceof ObjectNode objectNode)
         JSONTools.toEuclid(objectNode, rightHandTransformToChest.getValueAndModify());
      if (jsonNode.get("leftHandTransformToChest") instanceof ObjectNode objectNode)
         JSONTools.toEuclid(objectNode, leftHandTransformToChest.getValueAndModify());
      if (jsonNode.get("pelvisTransformToParent") instanceof ObjectNode objectNode)
         JSONTools.toEuclid(objectNode, pelvisTransformToParent.getValueAndModify());
      if (jsonNode.has("spinePitchDegrees"))
         setSpinePitchDegrees(jsonNode.get("spinePitchDegrees").asDouble());
      if (jsonNode.has("spineRollDegrees"))
         setSpineRollDegrees(jsonNode.get("spineRollDegrees").asDouble());
      if (jsonNode.has("spineYawDegrees"))
         setSpineYawDegrees(jsonNode.get("spineYawDegrees").asDouble());
      if (jsonNode.has("rightHandTrajectoryDuration"))
         setRightHandTrajectoryDuration(jsonNode.get("rightHandTrajectoryDuration").asDouble());
      if (jsonNode.has("leftHandTrajectoryDuration"))
         setLeftHandTrajectoryDuration(jsonNode.get("leftHandTrajectoryDuration").asDouble());
      if (jsonNode.has("spineTrajectoryDuration"))
         setSpineTrajectoryDuration(jsonNode.get("spineTrajectoryDuration").asDouble());
      if (jsonNode.has("pelvisTrajectoryDuration"))
         setPelvisTrajectoryDuration(jsonNode.get("pelvisTrajectoryDuration").asDouble());
      if (jsonNode.has("leftHandMask"))
         setLeftHandMask(jsonNode.get("leftHandMask").asInt());
      if (jsonNode.has("rightHandMask"))
         setRightHandMask(jsonNode.get("rightHandMask").asInt());
      if (jsonNode.has("spineMask"))
         setSpineMask(jsonNode.get("spineMask").asInt());
      if (jsonNode.has("pelvisMask"))
         setPelvisMask(jsonNode.get("pelvisMask").asInt());
      if (jsonNode.has("probabilityRightArmEnabled"))
         setProbabilityRightArmEnabled(jsonNode.get("probabilityRightArmEnabled").asDouble());

      if (jsonNode.has("probabilityLeftArmEnabled"))
         setProbabilityLeftArmEnabled(jsonNode.get("probabilityLeftArmEnabled").asDouble());

      if (jsonNode.has("probabilityTorsoEnabled"))
         setProbabilityTorsoEnabled(jsonNode.get("probabilityTorsoEnabled").asDouble());

      if (jsonNode.has("probabilityPelvisEnabled"))
         setProbabilityPelvisEnabled(jsonNode.get("probabilityPelvisEnabled").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();
      onDiskRandomizeGoToAction = randomizeGoToAction.getValue();
      onDiskRandomizeWholeBodyAction = randomizeWholeBodyAction.getValue();
      onDiskNumberOfRandomizations = numberOfRandomizations.getValue();
      onDiskWholeBodyRandomizationRequestID = wholeBodyRandomizationRequestID.getValue();
      onDiskRightHandTransformToChest.set(rightHandTransformToChest.getValueReadOnly());
      onDiskLeftHandTransformToChest.set(leftHandTransformToChest.getValueReadOnly());
      onDiskPelvisTransformToParent.set(pelvisTransformToParent.getValueReadOnly());
      onDiskSpinePitchDegrees = spinePitchDegrees.getValue();
      onDiskSpineRollDegrees = spineRollDegrees.getValue();
      onDiskSpineYawDegrees = spineYawDegrees.getValue();
      onDiskRightHandTrajectoryDuration = rightHandTrajectoryDuration.getValue();
      onDiskLeftHandTrajectoryDuration = leftHandTrajectoryDuration.getValue();
      onDiskSpineTrajectoryDuration = spineTrajectoryDuration.getValue();
      onDiskPelvisTrajectoryDuration = pelvisTrajectoryDuration.getValue();
      onDiskLeftHandMask = leftHandMask.getValue();
      onDiskRightHandMask = rightHandMask.getValue();
      onDiskSpineMask = spineMask.getValue();
      onDiskPelvisMask = pelvisMask.getValue();
      onDiskProbabilityRightArmEnabled = probabilityRightArmEnabled.getValue();
      onDiskProbabilityLeftArmEnabled = probabilityLeftArmEnabled.getValue();
      onDiskProbabilityTorsoEnabled = probabilityTorsoEnabled.getValue();
      onDiskProbabilityPelvisEnabled = probabilityPelvisEnabled.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();
      if (!isUndoAvailable())
         return;
      setRandomizeGoToActionEnabled(onDiskRandomizeGoToAction);
      setRandomizeWholeBodyActionEnabled(onDiskRandomizeWholeBodyAction);
      setNumberOfRandomizationsValue(onDiskNumberOfRandomizations);
      setWholeBodyRandomizationRequestID(onDiskWholeBodyRandomizationRequestID);
      rightHandTransformToChest.getValueAndModify().set(onDiskRightHandTransformToChest);
      leftHandTransformToChest.getValueAndModify().set(onDiskLeftHandTransformToChest);
      pelvisTransformToParent.getValueAndModify().set(onDiskPelvisTransformToParent);
      setSpinePitchDegrees(onDiskSpinePitchDegrees);
      setSpineRollDegrees(onDiskSpineRollDegrees);
      setSpineYawDegrees(onDiskSpineYawDegrees);
      setRightHandTrajectoryDuration(onDiskRightHandTrajectoryDuration);
      setLeftHandTrajectoryDuration(onDiskLeftHandTrajectoryDuration);
      setSpineTrajectoryDuration(onDiskSpineTrajectoryDuration);
      setPelvisTrajectoryDuration(onDiskPelvisTrajectoryDuration);
      setLeftHandMask(onDiskLeftHandMask);
      setRightHandMask(onDiskRightHandMask);
      setSpineMask(onDiskSpineMask);
      setPelvisMask(onDiskPelvisMask);
      setProbabilityRightArmEnabled(onDiskProbabilityRightArmEnabled);
      setProbabilityLeftArmEnabled(onDiskProbabilityLeftArmEnabled);
      setProbabilityTorsoEnabled(onDiskProbabilityTorsoEnabled);
      setProbabilityPelvisEnabled(onDiskProbabilityPelvisEnabled);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();
      unchanged &= randomizeGoToAction.getValue() == onDiskRandomizeGoToAction;
      unchanged &= randomizeWholeBodyAction.getValue() == onDiskRandomizeWholeBodyAction;
      unchanged &= numberOfRandomizations.getValue() == onDiskNumberOfRandomizations;
      unchanged &= wholeBodyRandomizationRequestID.getValue() == onDiskWholeBodyRandomizationRequestID;
      unchanged &= rightHandTransformToChest.getValueReadOnly().equals(onDiskRightHandTransformToChest);
      unchanged &= leftHandTransformToChest.getValueReadOnly().equals(onDiskLeftHandTransformToChest);
      unchanged &= pelvisTransformToParent.getValueReadOnly().equals(onDiskPelvisTransformToParent);
      unchanged &= spinePitchDegrees.getValue() == onDiskSpinePitchDegrees;
      unchanged &= spineRollDegrees.getValue() == onDiskSpineRollDegrees;
      unchanged &= spineYawDegrees.getValue() == onDiskSpineYawDegrees;
      unchanged &= rightHandTrajectoryDuration.getValue() == onDiskRightHandTrajectoryDuration;
      unchanged &= leftHandTrajectoryDuration.getValue() == onDiskLeftHandTrajectoryDuration;
      unchanged &= spineTrajectoryDuration.getValue() == onDiskSpineTrajectoryDuration;
      unchanged &= pelvisTrajectoryDuration.getValue() == onDiskPelvisTrajectoryDuration;
      unchanged &= leftHandMask.getValue() == onDiskLeftHandMask;
      unchanged &= rightHandMask.getValue() == onDiskRightHandMask;
      unchanged &= spineMask.getValue() == onDiskSpineMask;
      unchanged &= pelvisMask.getValue() == onDiskPelvisMask;
      unchanged &= probabilityRightArmEnabled.getValue() == onDiskProbabilityRightArmEnabled;
      unchanged &= probabilityLeftArmEnabled.getValue() == onDiskProbabilityLeftArmEnabled;
      unchanged &= probabilityTorsoEnabled.getValue() == onDiskProbabilityTorsoEnabled;
      unchanged &= probabilityPelvisEnabled.getValue() == onDiskProbabilityPelvisEnabled;
      return !unchanged;
   }

   public void toMessage(AI2RNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
      message.setRandomizeGoToAction(randomizeGoToAction.toMessage());
      message.setRandomizeWholeBodyAction(randomizeWholeBodyAction.toMessage());
      message.setNumberOfRandomizations(numberOfRandomizations.toMessage());
      message.setWholeBodyRandomizationRequestId(wholeBodyRandomizationRequestID.toMessage());
      rightHandTransformToChest.toMessage(message.getRightHandTransformToChest());
      leftHandTransformToChest.toMessage(message.getLeftHandTransformToChest());
      pelvisTransformToParent.toMessage(message.getPelvisTransformToParent());
      message.setSpinePitchDegrees(spinePitchDegrees.toMessage());
      message.setSpineRollDegrees(spineRollDegrees.toMessage());
      message.setSpineYawDegrees(spineYawDegrees.toMessage());
      message.setRightHandTrajectoryDuration(rightHandTrajectoryDuration.toMessage());
      message.setLeftHandTrajectoryDuration(leftHandTrajectoryDuration.toMessage());
      message.setSpineTrajectoryDuration(spineTrajectoryDuration.toMessage());
      message.setPelvisTrajectoryDuration(pelvisTrajectoryDuration.toMessage());
      message.setLeftHandMask(leftHandMask.toMessage());
      message.setRightHandMask(rightHandMask.toMessage());
      message.setSpineMask(spineMask.toMessage());
      message.setPelvisMask(pelvisMask.toMessage());
      message.setProbabilityOneEnabled(probabilityRightArmEnabled.toMessage());
      message.setProbabilityTwoEnabled(probabilityLeftArmEnabled.toMessage());
      message.setProbabilityThreeEnabled(probabilityTorsoEnabled.toMessage());
      message.setProbabilityFourEnabled(probabilityPelvisEnabled.toMessage());
   }

   public void fromMessage(AI2RNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
      randomizeGoToAction.fromMessage(message.getRandomizeGoToAction());
      randomizeWholeBodyAction.fromMessage(message.getRandomizeWholeBodyAction());
      numberOfRandomizations.fromMessage(message.getNumberOfRandomizations());
      wholeBodyRandomizationRequestID.fromMessage(message.getWholeBodyRandomizationRequestId());
      rightHandTransformToChest.fromMessage(message.getRightHandTransformToChest());
      leftHandTransformToChest.fromMessage(message.getLeftHandTransformToChest());
      pelvisTransformToParent.fromMessage(message.getPelvisTransformToParent());
      spinePitchDegrees.fromMessage(message.getSpinePitchDegrees());
      spineRollDegrees.fromMessage(message.getSpineRollDegrees());
      spineYawDegrees.fromMessage(message.getSpineYawDegrees());
      rightHandTrajectoryDuration.fromMessage(message.getRightHandTrajectoryDuration());
      leftHandTrajectoryDuration.fromMessage(message.getLeftHandTrajectoryDuration());
      spineTrajectoryDuration.fromMessage(message.getSpineTrajectoryDuration());
      pelvisTrajectoryDuration.fromMessage(message.getPelvisTrajectoryDuration());
      setLeftHandMask(message.getLeftHandMask());
      setRightHandMask(message.getRightHandMask());
      setSpineMask(message.getSpineMask());
      setPelvisMask(message.getPelvisMask());
      setProbabilityRightArmEnabled(message.getProbabilityOneEnabled());
      setProbabilityLeftArmEnabled(message.getProbabilityTwoEnabled());
      setProbabilityTorsoEnabled(message.getProbabilityThreeEnabled());
      setProbabilityPelvisEnabled(message.getProbabilityFourEnabled());
      setNumberOfRandomizationsValue(numberOfRandomizations.getValue());
   }

   public boolean getRandomizeGoToActionEnabled() { return randomizeGoToAction.getValue(); }
   public void setRandomizeGoToActionEnabled(boolean enabled) { randomizeGoToAction.setValue(enabled); }
   public boolean getRandomizeWholeBodyActionEnabled() { return randomizeWholeBodyAction.getValue(); }
   public void setRandomizeWholeBodyActionEnabled(boolean enabled) { randomizeWholeBodyAction.setValue(enabled); }
   public int getNumberOfRandomizationsValue() { return numberOfRandomizations.getValue(); }
   public void setNumberOfRandomizationsValue(int value) { numberOfRandomizations.setValue(Math.max(0, value)); }
   public CRDTBidirectionalBoolean getRandomizeGoToAction() { return randomizeGoToAction; }
   public CRDTBidirectionalInteger getNumberOfRandomizations() { return numberOfRandomizations; }
   public int getWholeBodyRandomizationRequestID() { return wholeBodyRandomizationRequestID.getValue(); }
   public void setWholeBodyRandomizationRequestID(int value) { wholeBodyRandomizationRequestID.setValue(Math.max(0, value)); }

   public double getSpinePitchDegrees() { return spinePitchDegrees.getValue(); }
   public void setSpinePitchDegrees(double value) { spinePitchDegrees.setValue(value); }
   public double getSpineRollDegrees() { return spineRollDegrees.getValue(); }
   public void setSpineRollDegrees(double value) { spineRollDegrees.setValue(value); }
   public double getSpineYawDegrees() { return spineYawDegrees.getValue(); }
   public void setSpineYawDegrees(double value) { spineYawDegrees.setValue(value); }

   public double getPelvisYawDegrees() { return Math.toDegrees(pelvisTransformToParent.getValueReadOnly().getRotation().getYaw()); }
   public void setPelvisYawDegrees(double value)
   {
      var rotation = pelvisTransformToParent.getValueAndModify().getRotation();
      rotation.setYawPitchRoll(Math.toRadians(value), rotation.getPitch(), rotation.getRoll());
   }
   public double getPelvisPitchDegrees() { return Math.toDegrees(pelvisTransformToParent.getValueReadOnly().getRotation().getPitch()); }
   public void setPelvisPitchDegrees(double value)
   {
      var rotation = pelvisTransformToParent.getValueAndModify().getRotation();
      rotation.setYawPitchRoll(rotation.getYaw(), Math.toRadians(value), rotation.getRoll());
   }
   public double getPelvisRollDegrees() { return Math.toDegrees(pelvisTransformToParent.getValueReadOnly().getRotation().getRoll()); }
   public void setPelvisRollDegrees(double value)
   {
      var rotation = pelvisTransformToParent.getValueAndModify().getRotation();
      rotation.setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), Math.toRadians(value));
   }
   public double getPelvisHeight() { return pelvisTransformToParent.getValueReadOnly().getTranslationZ(); }
   public void setPelvisHeight(double value) { pelvisTransformToParent.getValueAndModify().getTranslation().setZ(value); }

   public double getRightHandTrajectoryDuration() { return rightHandTrajectoryDuration.getValue(); }
   public void setRightHandTrajectoryDuration(double value) { rightHandTrajectoryDuration.setValue(Math.max(0.0, value)); }
   public double getLeftHandTrajectoryDuration() { return leftHandTrajectoryDuration.getValue(); }
   public void setLeftHandTrajectoryDuration(double value) { leftHandTrajectoryDuration.setValue(Math.max(0.0, value)); }
   public double getSpineTrajectoryDuration() { return spineTrajectoryDuration.getValue(); }
   public void setSpineTrajectoryDuration(double value) { spineTrajectoryDuration.setValue(Math.max(0.0, value)); }
   public double getPelvisTrajectoryDuration() { return pelvisTrajectoryDuration.getValue(); }
   public void setPelvisTrajectoryDuration(double value) { pelvisTrajectoryDuration.setValue(Math.max(0.0, value)); }
   public int getLeftHandMask() { return leftHandMask.getValue(); }
   public void setLeftHandMask(int value) { leftHandMask.setValue(value > 0 ? 1 : 0); }
   public int getRightHandMask() { return rightHandMask.getValue(); }
   public void setRightHandMask(int value) { rightHandMask.setValue(value > 0 ? 1 : 0); }
   public int getSpineMask() { return spineMask.getValue(); }
   public void setSpineMask(int value) { spineMask.setValue(value > 0 ? 1 : 0); }
   public int getPelvisMask() { return pelvisMask.getValue(); }
   public void setPelvisMask(int value) { pelvisMask.setValue(value > 0 ? 1 : 0); }
   public double getProbabilityRightArmEnabled() { return probabilityRightArmEnabled.getValue(); }
   public void setProbabilityRightArmEnabled(double value) { probabilityRightArmEnabled.setValue(clampProbability(value)); }
   public double getProbabilityLeftArmEnabled() { return probabilityLeftArmEnabled.getValue(); }
   public void setProbabilityLeftArmEnabled(double value) { probabilityLeftArmEnabled.setValue(clampProbability(value)); }
   public double getProbabilityTorsoEnabled() { return probabilityTorsoEnabled.getValue(); }
   public void setProbabilityTorsoEnabled(double value) { probabilityTorsoEnabled.setValue(clampProbability(value)); }
   public double getProbabilityPelvisEnabled() { return probabilityPelvisEnabled.getValue(); }
   public void setProbabilityPelvisEnabled(double value) { probabilityPelvisEnabled.setValue(clampProbability(value)); }

   private static double clampProbability(double value)
   {
      return Math.max(0.0, Math.min(1.0, value));
   }
}
