package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalString objectFrameName;
   private final CRDTUnidirectionalRigidBodyTransform screwAxisPoseInObjectFrame;
   /** The magnitude of the translation component */
   private final CRDTUnidirectionalDouble translation;
   /** The magnitude of the rotation component */
   private final CRDTUnidirectionalDouble rotation;
   private final CRDTUnidirectionalDouble maxLinearVelocity;
   private final CRDTUnidirectionalDouble maxAngularVelocity;
   private final CRDTUnidirectionalDouble maxForce;
   private final CRDTUnidirectionalDouble maxTorque;
   private final CRDTUnidirectionalDouble linearPositionWeight;
   private final CRDTUnidirectionalDouble angularPositionWeight;
   /** The operator specifies the estimated point of contact relative to the hand's control frame. */
   private final CRDTUnidirectionalRigidBodyTransform wrenchContactPoseInHandControlFrame;

   public ScrewPrimitiveActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      objectFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      screwAxisPoseInObjectFrame = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
      translation = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.1);
      rotation = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0);
      maxLinearVelocity = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.1);
      maxAngularVelocity = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.6);
      maxForce = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0); // 20 N is 4.5 pounds; lift a full two liter of soda
      maxTorque = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0); // 7 Nm is 5.2 ft-lbs; unscrewing a typical jar lid
      linearPositionWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, -1.0);
      angularPositionWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, -1.0);
      wrenchContactPoseInHandControlFrame = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("objectFrame", objectFrameName.getValue());
      JSONTools.toJSON(jsonNode, "screwAxisPose", screwAxisPoseInObjectFrame.getValueReadOnly());
      jsonNode.put("translation", translation.getValue());
      jsonNode.put("rotation", rotation.getValue());
      jsonNode.put("maxLinearVelocity", maxLinearVelocity.getValue());
      jsonNode.put("maxAngularVelocity", maxAngularVelocity.getValue());
      jsonNode.put("maxTorque", maxTorque.getValue());
      jsonNode.put("maxForce", maxForce.getValue());
      jsonNode.put("linearPositionWeight", linearPositionWeight.getValue());
      jsonNode.put("angularPositionWeight", angularPositionWeight.getValue());
      JSONTools.toJSON(jsonNode, "wrenchContactPose", wrenchContactPoseInHandControlFrame.getValueReadOnly());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      objectFrameName.setValue(jsonNode.get("objectFrame").textValue());
      JSONTools.toEuclid(jsonNode, "screwAxisPose", screwAxisPoseInObjectFrame.getValue());
      translation.setValue(jsonNode.get("translation").asDouble());
      rotation.setValue(jsonNode.get("rotation").asDouble());
      maxLinearVelocity.setValue(jsonNode.get("maxLinearVelocity").asDouble());
      maxAngularVelocity.setValue(jsonNode.get("maxAngularVelocity").asDouble());
      maxTorque.setValue(jsonNode.get("maxTorque").asDouble());
      maxForce.setValue(jsonNode.get("maxForce").asDouble());
      linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
      angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
      JSONTools.toEuclid(jsonNode, "wrenchContactPose", wrenchContactPoseInHandControlFrame.getValue());
   }

   public void toMessage(ScrewPrimitiveActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setObjectFrameName(objectFrameName.toMessage());
      screwAxisPoseInObjectFrame.toMessage(message.getScrewAxisPose());
      message.setRotation(rotation.toMessage());
      message.setTranslation(translation.toMessage());
      message.setMaxLinearVelocity(maxLinearVelocity.toMessage());
      message.setMaxAngularVelocity(maxAngularVelocity.toMessage());
      message.setMaxForce(maxForce.toMessage());
      message.setMaxTorque(maxTorque.toMessage());
      message.setLinearPositionWeight(linearPositionWeight.toMessage());
      message.setAngularPositionWeight(angularPositionWeight.toMessage());
      wrenchContactPoseInHandControlFrame.toMessage(message.getWrenchContactPose());
   }

   public void fromMessage(ScrewPrimitiveActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      objectFrameName.fromMessage(message.getObjectFrameNameAsString());
      screwAxisPoseInObjectFrame.fromMessage(message.getScrewAxisPose());
      rotation.fromMessage(message.getRotation());
      translation.fromMessage(message.getTranslation());
      maxLinearVelocity.fromMessage(message.getMaxLinearVelocity());
      maxAngularVelocity.fromMessage(message.getMaxAngularVelocity());
      maxForce.fromMessage(message.getMaxForce());
      maxTorque.fromMessage(message.getMaxTorque());
      linearPositionWeight.fromMessage(message.getLinearPositionWeight());
      angularPositionWeight.fromMessage(message.getAngularPositionWeight());
      wrenchContactPoseInHandControlFrame.fromMessage(message.getWrenchContactPose());
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

   public String getObjectFrameName()
   {
      return objectFrameName.getValue();
   }

   public void setObjectFrameName(String objectFrameName)
   {
      this.objectFrameName.setValue(objectFrameName);
   }

   public CRDTUnidirectionalRigidBodyTransform getScrewAxisPoseInObjectFrame()
   {
      return screwAxisPoseInObjectFrame;
   }

   public double getTranslation()
   {
      return translation.getValue();
   }

   public void setTranslation(double translation)
   {
      this.translation.setValue(translation);
   }

   public double getRotation()
   {
      return rotation.getValue();
   }

   public void setRotation(double rotation)
   {
      this.rotation.setValue(rotation);
   }

   public double getMaxLinearVelocity()
   {
      return maxLinearVelocity.getValue();
   }

   public void setMaxLinearVelocity(double maxLinearVelocity)
   {
      this.maxLinearVelocity.setValue(maxLinearVelocity);
   }

   public double getMaxAngularVelocity()
   {
      return maxAngularVelocity.getValue();
   }

   public void setMaxAngularVelocity(double maxAngularVelocity)
   {
      this.maxAngularVelocity.setValue(maxAngularVelocity);
   }

   public double getMaxTorque()
   {
      return maxTorque.getValue();
   }

   public void setMaxTorque(double maxTorque)
   {
      this.maxTorque.setValue(maxTorque);
   }

   public double getMaxForce()
   {
      return maxForce.getValue();
   }

   public void setMaxForce(double maxForce)
   {
      this.maxForce.setValue(maxForce);
   }

   public double getLinearPositionWeight()
   {
      return linearPositionWeight.getValue();
   }

   public void setLinearPositionWeight(double linearPositionWeight)
   {
      this.linearPositionWeight.setValue(linearPositionWeight);
   }

   public double getAngularPositionWeight()
   {
      return angularPositionWeight.getValue();
   }

   public void setAngularPositionWeight(double angularPositionWeight)
   {
      this.angularPositionWeight.setValue(angularPositionWeight);
   }

   public CRDTUnidirectionalRigidBodyTransform getWrenchContactPoseInHandControlFrame()
   {
      return wrenchContactPoseInHandControlFrame;
   }
}
