package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.RLWalkActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

/** Walks to a goal pose with the RL walking goal reacher. The goal is expressed in a parent frame. */
public class RLWalkActionDefinition extends ActionNodeDefinition
{
   /** 0 uses the controller speed limits */
   public static final double DEFAULT_MAX_FORWARD_SPEED = 0.0;
   public static final double DEFAULT_MAX_TURNING_SPEED = 0.0;
   public static final double DEFAULT_POSITION_PROXIMITY = 0.1;

   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalRigidBodyTransform goalToParentTransform;
   private final CRDTBidirectionalDouble maxForwardSpeed;
   private final CRDTBidirectionalDouble maxTurningSpeed;
   private final CRDTBidirectionalDouble positionProximity;
   private final CRDTBidirectionalEnumField<RLWalkHeadingMode> headingMode;

   // On disk fields
   private String onDiskParentFrameName;
   private final RigidBodyTransform onDiskGoalToParentTransform = new RigidBodyTransform();
   private double onDiskMaxForwardSpeed;
   private double onDiskMaxTurningSpeed;
   private double onDiskPositionProximity;
   private RLWalkHeadingMode onDiskHeadingMode;

   public RLWalkActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      parentFrameName = new CRDTBidirectionalString(this, "Walking");
      goalToParentTransform = new CRDTBidirectionalRigidBodyTransform(this);
      maxForwardSpeed = new CRDTBidirectionalDouble(this, DEFAULT_MAX_FORWARD_SPEED);
      maxTurningSpeed = new CRDTBidirectionalDouble(this, DEFAULT_MAX_TURNING_SPEED);
      positionProximity = new CRDTBidirectionalDouble(this, DEFAULT_POSITION_PROXIMITY);
      headingMode = new CRDTBidirectionalEnumField<>(this, RLWalkHeadingMode.FACE_GOAL);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("parentFrame", parentFrameName.getValue());
      JSONTools.toJSON(jsonNode, goalToParentTransform.getValueReadOnly());
      jsonNode.put("maxForwardSpeed", maxForwardSpeed.getValue());
      jsonNode.put("maxTurningSpeed", maxTurningSpeed.getValue());
      jsonNode.put("positionProximity", positionProximity.getValue());
      jsonNode.put("headingMode", headingMode.getValue().name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, goalToParentTransform.getValueAndModify());
      maxForwardSpeed.setValue(jsonNode.get("maxForwardSpeed").asDouble());
      maxTurningSpeed.setValue(jsonNode.get("maxTurningSpeed").asDouble());
      positionProximity.setValue(jsonNode.get("positionProximity").asDouble());
      headingMode.setValue(RLWalkHeadingMode.valueOf(jsonNode.get("headingMode").textValue()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskParentFrameName = parentFrameName.getValue();
      onDiskGoalToParentTransform.set(goalToParentTransform.getValueReadOnly());
      onDiskMaxForwardSpeed = maxForwardSpeed.getValue();
      onDiskMaxTurningSpeed = maxTurningSpeed.getValue();
      onDiskPositionProximity = positionProximity.getValue();
      onDiskHeadingMode = headingMode.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         parentFrameName.setValue(onDiskParentFrameName);
         goalToParentTransform.getValueAndModify().set(onDiskGoalToParentTransform);
         maxForwardSpeed.setValue(onDiskMaxForwardSpeed);
         maxTurningSpeed.setValue(onDiskMaxTurningSpeed);
         positionProximity.setValue(onDiskPositionProximity);
         headingMode.setValue(onDiskHeadingMode);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= goalToParentTransform.getValueReadOnly().equals(onDiskGoalToParentTransform);
      unchanged &= maxForwardSpeed.getValue() == onDiskMaxForwardSpeed;
      unchanged &= maxTurningSpeed.getValue() == onDiskMaxTurningSpeed;
      unchanged &= positionProximity.getValue() == onDiskPositionProximity;
      unchanged &= headingMode.getValue() == onDiskHeadingMode;

      return !unchanged;
   }

   public void toMessage(RLWalkActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(parentFrameName.toMessage());
      goalToParentTransform.toMessage(message.getGoalTransformToParent());
      message.setMaxForwardSpeed(maxForwardSpeed.toMessage());
      message.setMaxTurningSpeed(maxTurningSpeed.toMessage());
      message.setPositionProximity(positionProximity.toMessage());
      message.setHeadingMode(headingMode.toMessageOrdinal());
   }

   public void fromMessage(RLWalkActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      goalToParentTransform.fromMessage(message.getGoalTransformToParent());
      maxForwardSpeed.fromMessage(message.getMaxForwardSpeed());
      maxTurningSpeed.fromMessage(message.getMaxTurningSpeed());
      positionProximity.fromMessage(message.getPositionProximity());
      headingMode.fromMessageOrdinal(message.getHeadingMode(), RLWalkHeadingMode.values);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTBidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTBidirectionalRigidBodyTransform getGoalToParentTransform()
   {
      return goalToParentTransform;
   }

   public double getGoalX()
   {
      return goalToParentTransform.getValueReadOnly().getTranslationX();
   }

   public void setGoalX(double x)
   {
      goalToParentTransform.getValueAndModify().getTranslation().setX(x);
   }

   public double getGoalY()
   {
      return goalToParentTransform.getValueReadOnly().getTranslationY();
   }

   public void setGoalY(double y)
   {
      goalToParentTransform.getValueAndModify().getTranslation().setY(y);
   }

   public double getGoalYaw()
   {
      return goalToParentTransform.getValueReadOnly().getRotation().getYaw();
   }

   public void setGoalYaw(double yaw)
   {
      goalToParentTransform.getValueAndModify().getRotation().setToYawOrientation(yaw);
   }

   public double getMaxForwardSpeed()
   {
      return maxForwardSpeed.getValue();
   }

   public void setMaxForwardSpeed(double maxForwardSpeed)
   {
      this.maxForwardSpeed.setValue(maxForwardSpeed);
   }

   public double getMaxTurningSpeed()
   {
      return maxTurningSpeed.getValue();
   }

   public void setMaxTurningSpeed(double maxTurningSpeed)
   {
      this.maxTurningSpeed.setValue(maxTurningSpeed);
   }

   public double getPositionProximity()
   {
      return positionProximity.getValue();
   }

   public void setPositionProximity(double positionProximity)
   {
      this.positionProximity.setValue(positionProximity);
   }

   public RLWalkHeadingMode getHeadingMode()
   {
      return headingMode.getValue();
   }

   public void setHeadingMode(RLWalkHeadingMode headingMode)
   {
      this.headingMode.setValue(headingMode);
   }
}
