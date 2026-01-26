package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class ShapeContainsConditionDefinition
{
   public enum ContainsType
   {
      CONTAINS_FRAME,
      CONTAINS_POINTS;

      public static final ContainsType[] values = values();
   }

   private final CRDTBidirectionalEnumField<ContainsType> containsType;
   private final CRDTBidirectionalString frameName;
   private final CRDTBidirectionalInteger minPoints;
   private final CRDTBidirectionalString shapeParentFrameName;
   private final CRDTBidirectionalRigidBodyTransform shapeTransformToParent;
   private final CRDTBidirectionalDouble sphereRadius;

   private ContainsType onDiskContainsType;
   private String onDiskFrameName;
   private int onDiskMinPoints;
   private String onDiskShapeParentFrameName;
   private final RigidBodyTransform onDiskShapeTransformToParent = new RigidBodyTransform();
   private double onDiskSphereRadius;

   public ShapeContainsConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      containsType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, ContainsType.CONTAINS_FRAME);
      frameName = new CRDTBidirectionalString(latestTimestampModifiable, ReferenceFrame.getWorldFrame().getName());
      minPoints = new CRDTBidirectionalInteger(latestTimestampModifiable, 0);
      shapeParentFrameName = new CRDTBidirectionalString(latestTimestampModifiable, ReferenceFrame.getWorldFrame().getName());
      shapeTransformToParent = new CRDTBidirectionalRigidBodyTransform(latestTimestampModifiable);
      sphereRadius = new CRDTBidirectionalDouble(latestTimestampModifiable, 0.5);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("containsType", containsType.getValue().toString());
      jsonNode.put("frameName", frameName.getValue());
      jsonNode.put("minPoints", minPoints.getValue());
      jsonNode.put("shapeParentFrameName", shapeParentFrameName.getValue());
      JSONTools.toJSON(jsonNode, "shapeTransformToParent", shapeTransformToParent.getValueReadOnly());
      jsonNode.put("sphereRadius", sphereRadius.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      containsType.setValue(ContainsType.valueOf(jsonNode.get("containsType").textValue()));
      frameName.setValue(jsonNode.get("frameName").textValue());
      minPoints.setValue(jsonNode.get("minPoints").asInt());
      shapeParentFrameName.setValue(jsonNode.get("shapeParentFrameName").textValue());
      JSONTools.toEuclid(jsonNode, "shapeTransformToParent", shapeTransformToParent.getValueAndModify());
      sphereRadius.setValue(jsonNode.get("sphereRadius").asDouble());
   }

   public void setOnDiskFields()
   {
      onDiskContainsType = containsType.getValue();
      onDiskFrameName = frameName.getValue();
      onDiskMinPoints = minPoints.getValue();
      onDiskShapeParentFrameName = shapeParentFrameName.getValue();
      onDiskShapeTransformToParent.set(shapeTransformToParent.getValueReadOnly());
      onDiskSphereRadius = sphereRadius.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      containsType.setValue(onDiskContainsType);
      frameName.setValue(onDiskFrameName);
      minPoints.setValue(onDiskMinPoints);
      shapeParentFrameName.setValue(onDiskShapeParentFrameName);
      shapeTransformToParent.getValueAndModify().set(onDiskShapeTransformToParent);
      sphereRadius.setValue(onDiskSphereRadius);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= containsType.getValue() == onDiskContainsType;
      unchanged &= frameName.getValue().equals(onDiskFrameName);
      unchanged &= minPoints.getValue() == onDiskMinPoints;
      unchanged &= shapeParentFrameName.getValue().equals(onDiskShapeParentFrameName);
      unchanged &= shapeTransformToParent.getValueReadOnly().equals(onDiskShapeTransformToParent);
      unchanged &= sphereRadius.getValue() == onDiskSphereRadius;

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setContainsType(containsType.toMessageOrdinal());
      message.setFrameName(frameName.toMessage());
      message.setMinPoints(minPoints.toMessage());
      message.setShapeParentFrameName(shapeParentFrameName.toMessage());
      shapeTransformToParent.toMessage(message.getShapeTransformToParent());
      message.setSphereRadius((float) sphereRadius.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      containsType.fromMessageOrdinal(message.getContainsType(), ContainsType.values);
      frameName.fromMessage(message.getFrameNameAsString());
      minPoints.fromMessage((int) message.getMinPoints());
      shapeParentFrameName.fromMessage(message.getShapeParentFrameNameAsString());
      shapeTransformToParent.fromMessage(message.getShapeTransformToParent());
      sphereRadius.fromMessage(message.getSphereRadius());
   }

   public ContainsType getContainsType()
   {
      return containsType.getValue();
   }

   public void setContainsType(ContainsType containsType)
   {
      this.containsType.setValue(containsType);
   }

   public String getFrameName()
   {
      return frameName.getValue();
   }

   public void setFrameName(String frameName)
   {
      this.frameName.setValue(frameName);
   }

   public int getMinPoints()
   {
      return minPoints.getValue();
   }

   public void setMinPoints(int minPoints)
   {
      this.minPoints.setValue(minPoints);
   }

   public String getShapeParentFrameName()
   {
      return shapeParentFrameName.getValue();
   }

   public void setShapeParentFrameName(String shapeParentFrameName)
   {
      this.shapeParentFrameName.setValue(shapeParentFrameName);
   }

   public CRDTBidirectionalString getShapeParentFrameNameCRDT()
   {
      return shapeParentFrameName;
   }

   public CRDTBidirectionalRigidBodyTransform getShapeTransformToParent()
   {
      return shapeTransformToParent;
   }

   public double getSphereRadius()
   {
      return sphereRadius.getValue();
   }

   public void setSphereRadius(double sphereRadius)
   {
      this.sphereRadius.setValue(sphereRadius);
   }
}
