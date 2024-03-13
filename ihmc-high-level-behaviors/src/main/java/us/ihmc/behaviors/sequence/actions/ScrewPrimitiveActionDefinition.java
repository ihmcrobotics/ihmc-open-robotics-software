package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public static final double DEFAULT_POSITION_ERROR_TOLERANCE = 0.15;
   public static final double DEFAULT_ORIENTATION_ERROR_TOLERANCE = Math.toRadians(10.0);

   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalString objectFrameName;
   private final CRDTUnidirectionalRigidBodyTransform screwAxisPoseInObjectFrame;
   /** The magnitude of the translation component */
   private final CRDTUnidirectionalDouble translation;
   /** The magnitude of the rotation component */
   private final CRDTUnidirectionalDouble rotation;
   private final CRDTUnidirectionalDouble maxLinearVelocity;
   private final CRDTUnidirectionalDouble maxAngularVelocity;
   private final CRDTUnidirectionalBoolean jointspaceOnly;
   private final CRDTUnidirectionalDouble linearPositionWeight;
   private final CRDTUnidirectionalDouble angularPositionWeight;
   private final CRDTUnidirectionalDouble jointspaceWeight;
   private final CRDTUnidirectionalDouble positionErrorTolerance;
   private final CRDTUnidirectionalDouble orientationErrorTolerance;

   // On disk fields
   private RobotSide onDiskSide;
   private String onDiskObjectFrameName;
   private final RigidBodyTransform onDiskScrewAxisPoseInObjectFrame = new RigidBodyTransform();
   private double onDiskTranslation;
   private double onDiskRotation;
   private double onDiskMaxLinearVelocity;
   private double onDiskMaxAngularVelocity;
   private boolean onDiskJointspaceOnly;
   private double onDiskLinearPositionWeight;
   private double onDiskAngularPositionWeight;
   private double onDiskJointspaceWeight;
   private double onDiskPositionErrorTolerance;
   private double onDiskOrientationErrorTolerance;

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
      jointspaceOnly = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, true); // Jointspace only works best for now
      linearPositionWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, -1.0);
      angularPositionWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, -1.0);
      jointspaceWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, -1.0);
      positionErrorTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_POSITION_ERROR_TOLERANCE);
      orientationErrorTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_ORIENTATION_ERROR_TOLERANCE);
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
      jsonNode.put("jointspaceOnly", jointspaceOnly.getValue());
      jsonNode.put("linearPositionWeight", linearPositionWeight.getValue());
      jsonNode.put("angularPositionWeight", angularPositionWeight.getValue());
      jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
      jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
      jsonNode.put("orientationErrorToleranceDegrees", Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
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
      jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
      linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
      angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
      jointspaceWeight.setValue(jsonNode.get("jointspaceWeight").asDouble());
      positionErrorTolerance.setValue(jsonNode.get("positionErrorTolerance").asDouble());
      orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskObjectFrameName = objectFrameName.getValue();
      onDiskScrewAxisPoseInObjectFrame.set(screwAxisPoseInObjectFrame.getValueReadOnly());
      onDiskTranslation = translation.getValue();
      onDiskRotation = rotation.getValue();
      onDiskMaxLinearVelocity = maxLinearVelocity.getValue();
      onDiskMaxAngularVelocity = maxAngularVelocity.getValue();
      onDiskJointspaceOnly = jointspaceOnly.getValue();
      onDiskLinearPositionWeight = linearPositionWeight.getValue();
      onDiskAngularPositionWeight = angularPositionWeight.getValue();
      onDiskJointspaceWeight = jointspaceWeight.getValue();
      onDiskPositionErrorTolerance = positionErrorTolerance.getValue();
      onDiskOrientationErrorTolerance = orientationErrorTolerance.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      objectFrameName.setValue(onDiskObjectFrameName);
      screwAxisPoseInObjectFrame.getValue().set(onDiskScrewAxisPoseInObjectFrame);
      translation.setValue(onDiskTranslation);
      rotation.setValue(onDiskRotation);
      maxLinearVelocity.setValue(onDiskMaxLinearVelocity);
      maxAngularVelocity.setValue(onDiskMaxAngularVelocity);
      jointspaceOnly.setValue(onDiskJointspaceOnly);
      linearPositionWeight.setValue(onDiskLinearPositionWeight);
      angularPositionWeight.setValue(onDiskAngularPositionWeight);
      jointspaceWeight.setValue(onDiskJointspaceWeight);
      positionErrorTolerance.setValue(onDiskPositionErrorTolerance);
      orientationErrorTolerance.setValue(onDiskOrientationErrorTolerance);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= objectFrameName.getValue().equals(onDiskObjectFrameName);
      unchanged &= screwAxisPoseInObjectFrame.getValueReadOnly().equals(onDiskScrewAxisPoseInObjectFrame);
      unchanged &= translation.getValue() == onDiskTranslation;
      unchanged &= rotation.getValue() == onDiskRotation;
      unchanged &= maxLinearVelocity.getValue() == onDiskMaxLinearVelocity;
      unchanged &= maxAngularVelocity.getValue() == onDiskMaxAngularVelocity;
      unchanged &= jointspaceOnly.getValue() == onDiskJointspaceOnly;
      unchanged &= linearPositionWeight.getValue() == onDiskLinearPositionWeight;
      unchanged &= angularPositionWeight.getValue() == onDiskAngularPositionWeight;
      unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;
      unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
      unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;

      return !unchanged;
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
      message.setJointspaceOnly(jointspaceOnly.toMessage());
      message.setLinearPositionWeight(linearPositionWeight.toMessage());
      message.setAngularPositionWeight(angularPositionWeight.toMessage());
      message.setJointspaceWeight(jointspaceWeight.toMessage());
      message.setPositionErrorTolerance(positionErrorTolerance.toMessage());
      message.setOrientationErrorTolerance(orientationErrorTolerance.toMessage());
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
      jointspaceOnly.fromMessage(message.getJointspaceOnly());
      linearPositionWeight.fromMessage(message.getLinearPositionWeight());
      angularPositionWeight.fromMessage(message.getAngularPositionWeight());
      jointspaceWeight.fromMessage(message.getJointspaceWeight());
      positionErrorTolerance.fromMessage(message.getPositionErrorTolerance());
      orientationErrorTolerance.fromMessage(message.getOrientationErrorTolerance());
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

   public boolean getJointspaceOnly()
   {
      return jointspaceOnly.getValue();
   }

   public void setJointspaceOnly(boolean jointspaceOnly)
   {
      this.jointspaceOnly.setValue(jointspaceOnly);
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

   public double getJointspaceWeight()
   {
      return jointspaceWeight.getValue();
   }

   public void setJointspaceWeight(double jointspaceWeight)
   {
      this.jointspaceWeight.setValue(jointspaceWeight);
   }

   public double getPositionErrorTolerance()
   {
      return positionErrorTolerance.getValue();
   }

   public void setPositionErrorTolerance(double positionErrorTolerance)
   {
      this.positionErrorTolerance.setValue(positionErrorTolerance);
   }

   public double getOrientationErrorTolerance()
   {
      return orientationErrorTolerance.getValue();
   }

   public void setOrientationErrorTolerance(double orientationErrorTolerance)
   {
      this.orientationErrorTolerance.setValue(orientationErrorTolerance);
   }
}
