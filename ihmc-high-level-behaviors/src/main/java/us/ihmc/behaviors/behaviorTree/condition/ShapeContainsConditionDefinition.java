package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.IntNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
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
   private final CRDTBidirectionalInteger maxPoints;
   private final CRDTBidirectionalString shapeParentFrameName;
   private final CRDTBidirectionalRigidBodyTransform shapeTransformToParent;
   private final CRDTBidirectionalDouble sphereRadius;
   private final CRDTBidirectionalBoolean checkColor;
   private final CRDTBidirectionalInteger hueMin;
   private final CRDTBidirectionalInteger hueMax;
   private final CRDTBidirectionalInteger saturationMin;
   private final CRDTBidirectionalInteger saturationMax;
   private final CRDTBidirectionalInteger valueMin;
   private final CRDTBidirectionalInteger valueMax;

   private ContainsType onDiskContainsType;
   private String onDiskFrameName;
   private int onDiskMinPoints;
   private int onDiskMaxPoints;
   private String onDiskShapeParentFrameName;
   private final RigidBodyTransform onDiskShapeTransformToParent = new RigidBodyTransform();
   private double onDiskSphereRadius;
   private boolean onDiskCheckColor;
   private int onDiskHueMin;
   private int onDiskHueMax;
   private int onDiskSaturationMin;
   private int onDiskSaturationMax;
   private int onDiskValueMin;
   private int onDiskValueMax;

   public ShapeContainsConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      containsType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, ContainsType.CONTAINS_FRAME);
      frameName = new CRDTBidirectionalString(latestTimestampModifiable, "Chest");
      minPoints = new CRDTBidirectionalInteger(latestTimestampModifiable, 0);
      maxPoints = new CRDTBidirectionalInteger(latestTimestampModifiable, 2000000000);
      shapeParentFrameName = new CRDTBidirectionalString(latestTimestampModifiable, "Chest");
      shapeTransformToParent = new CRDTBidirectionalRigidBodyTransform(latestTimestampModifiable);
      sphereRadius = new CRDTBidirectionalDouble(latestTimestampModifiable, 0.5);
      checkColor = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      hueMin = new CRDTBidirectionalInteger(latestTimestampModifiable, 30);
      hueMax = new CRDTBidirectionalInteger(latestTimestampModifiable, 40);
      saturationMin = new CRDTBidirectionalInteger(latestTimestampModifiable, 20);
      saturationMax = new CRDTBidirectionalInteger(latestTimestampModifiable, 255);
      valueMin = new CRDTBidirectionalInteger(latestTimestampModifiable, 80);
      valueMax = new CRDTBidirectionalInteger(latestTimestampModifiable, 255);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("containsType", containsType.getValue().toString());
      jsonNode.put("frameName", frameName.getValue());
      if (containsType.getValue() == ContainsType.CONTAINS_POINTS)
      {
         jsonNode.put("minPoints", minPoints.getValue());
         jsonNode.put("maxPoints", maxPoints.getValue());
         jsonNode.put("checkColor", checkColor.getValue());
         if (checkColor.getValue())
         {
            jsonNode.put("hueMin", hueMin.getValue());
            jsonNode.put("hueMax", hueMax.getValue());
            jsonNode.put("saturationMin", saturationMin.getValue());
            jsonNode.put("saturationMax", saturationMax.getValue());
            jsonNode.put("valueMin", valueMin.getValue());
            jsonNode.put("valueMax", valueMax.getValue());
         }
      }
      jsonNode.put("shapeParentFrameName", shapeParentFrameName.getValue());
      JSONTools.toJSON(jsonNode.putObject("shapeTransformToParent"), shapeTransformToParent.getValueReadOnly());
      jsonNode.put("sphereRadius", JSONTools.toJsonMeters(sphereRadius.getValue()));
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      containsType.setValue(ContainsType.valueOf(jsonNode.get("containsType").textValue()));
      frameName.setValue(jsonNode.get("frameName").textValue());
      if (jsonNode.get("minPoints") instanceof IntNode intNode)
         minPoints.setValue(intNode.asInt());
      if (jsonNode.get("maxPoints") instanceof IntNode intNode)
         maxPoints.setValue(intNode.asInt());
      if (jsonNode.get("checkColor") != null)
         checkColor.setValue(jsonNode.get("checkColor").booleanValue());
      if (checkColor.getValue())
      {
         if (jsonNode.get("hueMin") instanceof IntNode intNode)
            hueMin.setValue(intNode.asInt());
         if (jsonNode.get("hueMax") instanceof IntNode intNode)
            hueMax.setValue(intNode.asInt());
         if (jsonNode.get("saturationMin") instanceof IntNode intNode)
            saturationMin.setValue(intNode.asInt());
         if (jsonNode.get("saturationMax") instanceof IntNode intNode)
            saturationMax.setValue(intNode.asInt());
         if (jsonNode.get("valueMin") instanceof IntNode intNode)
            valueMin.setValue(intNode.asInt());
         if (jsonNode.get("valueMax") instanceof IntNode intNode)
            valueMax.setValue(intNode.asInt());
      }
      shapeParentFrameName.setValue(jsonNode.get("shapeParentFrameName").textValue());
      if (jsonNode.get("shapeTransformToParent") instanceof ObjectNode objectNode)
         JSONTools.toEuclid(objectNode, shapeTransformToParent.getValueAndModify());
      sphereRadius.setValue(jsonNode.get("sphereRadius").asDouble());
   }

   public void setOnDiskFields()
   {
      onDiskContainsType = containsType.getValue();
      onDiskFrameName = frameName.getValue();
      onDiskMinPoints = minPoints.getValue();
      onDiskMaxPoints = maxPoints.getValue();
      onDiskShapeParentFrameName = shapeParentFrameName.getValue();
      onDiskShapeTransformToParent.set(shapeTransformToParent.getValueReadOnly());
      onDiskSphereRadius = sphereRadius.getValue();
      onDiskCheckColor = checkColor.getValue();
      onDiskHueMin = hueMin.getValue();
      onDiskHueMax = hueMax.getValue();
      onDiskSaturationMin = saturationMin.getValue();
      onDiskSaturationMax = saturationMax.getValue();
      onDiskValueMin = valueMin.getValue();
      onDiskValueMax = valueMax.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      containsType.setValue(onDiskContainsType);
      frameName.setValue(onDiskFrameName);
      minPoints.setValue(onDiskMinPoints);
      maxPoints.setValue(onDiskMaxPoints);
      shapeParentFrameName.setValue(onDiskShapeParentFrameName);
      shapeTransformToParent.getValueAndModify().set(onDiskShapeTransformToParent);
      sphereRadius.setValue(onDiskSphereRadius);
      checkColor.setValue(onDiskCheckColor);
      hueMin.setValue(onDiskHueMin);
      hueMax.setValue(onDiskHueMax);
      saturationMin.setValue(onDiskSaturationMin);
      saturationMax.setValue(onDiskSaturationMax);
      valueMin.setValue(onDiskValueMin);
      valueMax.setValue(onDiskValueMax);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= containsType.getValue() == onDiskContainsType;
      unchanged &= frameName.getValue().equals(onDiskFrameName);
      unchanged &= minPoints.getValue() == onDiskMinPoints;
      unchanged &= maxPoints.getValue() == onDiskMaxPoints;
      unchanged &= shapeParentFrameName.getValue().equals(onDiskShapeParentFrameName);
      unchanged &= shapeTransformToParent.getValueReadOnly().equals(onDiskShapeTransformToParent);
      unchanged &= sphereRadius.getValue() == onDiskSphereRadius;
      unchanged &= checkColor.getValue() == onDiskCheckColor;
      unchanged &= hueMin.getValue() == onDiskHueMin;
      unchanged &= hueMax.getValue() == onDiskHueMax;
      unchanged &= saturationMin.getValue() == onDiskSaturationMin;
      unchanged &= saturationMax.getValue() == onDiskSaturationMax;
      unchanged &= valueMin.getValue() == onDiskValueMin;
      unchanged &= valueMax.getValue() == onDiskValueMax;

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setShapeContainsType(containsType.toMessageOrdinal());
      message.setFrameName(frameName.toMessage());
      message.setMinPoints(minPoints.toMessage());
      message.setMaxPoints(maxPoints.toMessage());
      message.setShapeParentFrameName(shapeParentFrameName.toMessage());
      shapeTransformToParent.toMessage(message.getShapeTransformToParent());
      message.setSphereRadius((float) sphereRadius.toMessage());
      message.setCheckColor(checkColor.toMessage());
      message.setHueMin((short) hueMin.toMessage());
      message.setHueMax((short) hueMax.toMessage());
      message.setSaturationMin((short) saturationMin.toMessage());
      message.setSaturationMax((short) saturationMax.toMessage());
      message.setValueMin((short) valueMin.toMessage());
      message.setValueMax((short) valueMax.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      containsType.fromMessageOrdinal(message.getShapeContainsType(), ContainsType.values);
      frameName.fromMessage(message.getFrameNameAsString());
      minPoints.fromMessage((int) message.getMinPoints());
      maxPoints.fromMessage((int) message.getMaxPoints());
      shapeParentFrameName.fromMessage(message.getShapeParentFrameNameAsString());
      shapeTransformToParent.fromMessage(message.getShapeTransformToParent());
      sphereRadius.fromMessage(message.getSphereRadius());
      checkColor.fromMessage(message.getCheckColor());
      hueMin.fromMessage(message.getHueMin());
      hueMax.fromMessage(message.getHueMax());
      saturationMin.fromMessage(message.getSaturationMin());
      saturationMax.fromMessage(message.getSaturationMax());
      valueMin.fromMessage(message.getValueMin());
      valueMax.fromMessage(message.getValueMax());
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

   public int getMaxPoints()
   {
      return maxPoints.getValue();
   }

   public void setMaxPoints(int maxPoints)
   {
      this.maxPoints.setValue(maxPoints);
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

   public boolean getCheckColor()
   {
      return checkColor.getValue();
   }

   public void setCheckColor(boolean checkColor)
   {
      this.checkColor.setValue(checkColor);
   }

   public int getHueMin()
   {
      return hueMin.getValue();
   }

   public void setHueMin(int hueMin)
   {
      this.hueMin.setValue(hueMin);
   }

   public int getHueMax()
   {
      return hueMax.getValue();
   }

   public void setHueMax(int hueMax)
   {
      this.hueMax.setValue(hueMax);
   }

   public int getSaturationMin()
   {
      return saturationMin.getValue();
   }

   public void setSaturationMin(int saturationMin)
   {
      this.saturationMin.setValue(saturationMin);
   }

   public int getSaturationMax()
   {
      return saturationMax.getValue();
   }

   public void setSaturationMax(int saturationMax)
   {
      this.saturationMax.setValue(saturationMax);
   }

   public int getValueMin()
   {
      return valueMin.getValue();
   }

   public void setValueMin(int valueMin)
   {
      this.valueMin.setValue(valueMin);
   }

   public int getValueMax()
   {
      return valueMax.getValue();
   }

   public void setValueMax(int valueMax)
   {
      this.valueMax.setValue(valueMax);
   }
}
