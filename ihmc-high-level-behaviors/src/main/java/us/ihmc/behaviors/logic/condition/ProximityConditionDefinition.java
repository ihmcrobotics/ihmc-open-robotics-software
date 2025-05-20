package us.ihmc.behaviors.logic.condition;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class ProximityConditionDefinition
{
   public enum DistanceType
   {
      XYZ,
      XY,
      Z;

      public static final DistanceType[] values = values();
   }

   private final CRDTBidirectionalEnumField<DistanceType> type;

   private final CRDTBidirectionalString objectFrameName;
   private final CRDTBidirectionalString referenceFrameName;
   private final CRDTBidirectionalDouble maxDistanceToObject;

   private String onDiskObjectFrameName;
   private String onDiskReferenceFrameName;
   private double onDiskDistanceToObject;
   private DistanceType onDiskType;

   public ProximityConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      type = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, DistanceType.XYZ);
      objectFrameName = new CRDTBidirectionalString(latestTimestampModifiable, ReferenceFrame.getWorldFrame().getName());
      referenceFrameName = new CRDTBidirectionalString(latestTimestampModifiable, ReferenceFrame.getWorldFrame().getName());
      maxDistanceToObject = new CRDTBidirectionalDouble(latestTimestampModifiable, 1.0);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("objectFrameName", objectFrameName.getValue());
      jsonNode.put("referenceFrameName", referenceFrameName.getValue());
      jsonNode.put("maxDistanceToObject", maxDistanceToObject.getValue());
      jsonNode.put("distanceType", type.getValue().toString());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      objectFrameName.setValue(jsonNode.get("objectFrameName").textValue());
      referenceFrameName.setValue(jsonNode.get("referenceFrameName").textValue());
      maxDistanceToObject.setValue(jsonNode.get("maxDistanceToObject").asDouble());
      type.setValue(DistanceType.valueOf(jsonNode.get("distanceType").textValue()));
   }

   public void setOnDiskFields()
   {
      onDiskObjectFrameName = objectFrameName.getValue();
      onDiskReferenceFrameName = referenceFrameName.getValue();
      onDiskDistanceToObject = maxDistanceToObject.getValue();
      onDiskType = type.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      type.setValue(onDiskType);
      objectFrameName.setValue(onDiskObjectFrameName);
      referenceFrameName.setValue(onDiskReferenceFrameName);
      maxDistanceToObject.setValue(onDiskDistanceToObject);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= type.getValue() == onDiskType;
      unchanged &= objectFrameName.getValue().equals(onDiskObjectFrameName);
      unchanged &= referenceFrameName.getValue().equals(onDiskReferenceFrameName);
      unchanged &= maxDistanceToObject.getValue() == (onDiskDistanceToObject);

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setDistanceType((byte) type.toMessage().ordinal());
      message.setObjectFrameName(objectFrameName.toMessage());
      message.setReferenceFrameName(referenceFrameName.toMessage());
      message.setDistanceToObject(maxDistanceToObject.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      type.fromMessage(DistanceType.values()[message.getDistanceType()]);
      objectFrameName.fromMessage(message.getObjectFrameNameAsString());
      referenceFrameName.fromMessage(message.getReferenceFrameNameAsString());
      maxDistanceToObject.fromMessage(message.getDistanceToObject());
   }

   public void setObjectFrameName(String objectFrameName)
   {
      this.objectFrameName.setValue(objectFrameName);
   }

   public void setReferenceFrameName(String referenceFrameName)
   {
      this.referenceFrameName.setValue(referenceFrameName);
   }

   public void setMaxDistanceToObject(double distance)
   {
      this.maxDistanceToObject.setValue(distance);
   }

   public String getObjectFrameName()
   {
      return objectFrameName.getValue();
   }

   public String getReferenceFrameName()
   {
      return referenceFrameName.getValue();
   }

   public double getMaxDistanceToObject()
   {
      return maxDistanceToObject.getValue();
   }

   public CRDTBidirectionalString getCRDTObjectFrameName()
   {
      return objectFrameName;
   }

   public CRDTBidirectionalString getCRDTReferenceFrameName()
   {
      return referenceFrameName;
   }

   public CRDTBidirectionalDouble getCRDTMaxDistanceToObject()
   {
      return maxDistanceToObject;
   }

   public CRDTBidirectionalEnumField<DistanceType> getType()
   {
      return type;
   }
}
