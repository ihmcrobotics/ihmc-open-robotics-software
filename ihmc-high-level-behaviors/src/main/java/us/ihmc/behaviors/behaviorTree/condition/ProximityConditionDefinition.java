package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.ConditionNodeDefinitionMessage;
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

   private final CRDTBidirectionalEnumField<DistanceType> distanceType;

   private final CRDTBidirectionalString frameNameA;
   private final CRDTBidirectionalString frameNameB;
   private final CRDTBidirectionalDouble minDistance;
   private final CRDTBidirectionalDouble maxDistance;
   private final CRDTBidirectionalDouble timeout;

   private DistanceType onDiskDistanceType;
   private String onDiskFrameNameA;
   private String onDiskFrameNameB;
   private double onDiskMinDistance;
   private double onDiskMaxDistance;
   private double onDiskTimeout;

   public ProximityConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      distanceType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, DistanceType.XYZ);
      frameNameA = new CRDTBidirectionalString(latestTimestampModifiable, "Pelvis");
      frameNameB = new CRDTBidirectionalString(latestTimestampModifiable, "Chest");
      minDistance = new CRDTBidirectionalDouble(latestTimestampModifiable, 0.0);
      maxDistance = new CRDTBidirectionalDouble(latestTimestampModifiable, 1.0);
      timeout = new CRDTBidirectionalDouble(latestTimestampModifiable, 5.0);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("distanceType", distanceType.getValue().toString());
      jsonNode.put("frameNameA", frameNameA.getValue());
      jsonNode.put("frameNameB", frameNameB.getValue());
      jsonNode.put("minDistance", minDistance.getValue());
      jsonNode.put("maxDistance", maxDistance.getValue());
      jsonNode.put("timeout", timeout.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      distanceType.setValue(DistanceType.valueOf(jsonNode.get("distanceType").textValue()));
      frameNameA.setValue(jsonNode.get("frameNameA").textValue());
      frameNameB.setValue(jsonNode.get("frameNameB").textValue());
      minDistance.setValue(jsonNode.get("minDistance").asDouble());
      maxDistance.setValue(jsonNode.get("maxDistance").asDouble());
      timeout.setValue(jsonNode.get("timeout").asDouble());
   }

   public void setOnDiskFields()
   {
      onDiskDistanceType = distanceType.getValue();
      onDiskFrameNameA = frameNameA.getValue();
      onDiskFrameNameB = frameNameB.getValue();
      onDiskMinDistance = minDistance.getValue();
      onDiskMaxDistance = maxDistance.getValue();
      onDiskTimeout = timeout.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      distanceType.setValue(onDiskDistanceType);
      frameNameA.setValue(onDiskFrameNameA);
      frameNameB.setValue(onDiskFrameNameB);
      minDistance.setValue(onDiskMinDistance);
      maxDistance.setValue(onDiskMaxDistance);
      timeout.setValue(onDiskTimeout);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= distanceType.getValue() == onDiskDistanceType;
      unchanged &= frameNameA.getValue().equals(onDiskFrameNameA);
      unchanged &= frameNameB.getValue().equals(onDiskFrameNameB);
      unchanged &= minDistance.getValue() == (onDiskMinDistance);
      unchanged &= maxDistance.getValue() == (onDiskMaxDistance);
      unchanged &= timeout.getValue() == (onDiskTimeout);

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setDistanceType(distanceType.toMessageOrdinal());
      message.setFrameNameA(frameNameA.toMessage());
      message.setFrameNameB(frameNameB.toMessage());
      message.setMinDistance(minDistance.toMessage());
      message.setMaxDistance(maxDistance.toMessage());
      message.setTimeout(timeout.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      distanceType.fromMessageOrdinal(message.getDistanceType(), DistanceType.values);
      frameNameA.fromMessage(message.getFrameNameAAsString());
      frameNameB.fromMessage(message.getFrameNameBAsString());
      minDistance.fromMessage(message.getMinDistance());
      maxDistance.fromMessage(message.getMaxDistance());
      timeout.fromMessage(message.getTimeout());
   }

   public void setFrameNameA(String frameNameA)
   {
      this.frameNameA.setValue(frameNameA);
   }

   public void setFrameNameB(String frameNameB)
   {
      this.frameNameB.setValue(frameNameB);
   }

   public void setTimeout(double time)
   {
      this.timeout.setValue(time);
   }

   public String getFrameNameA()
   {
      return frameNameA.getValue();
   }

   public String getFrameNameB()
   {
      return frameNameB.getValue();
   }

   public double getMaxDistance()
   {
      return maxDistance.getValue();
   }

   public void setMaxDistance(double maxDistance)
   {
      this.maxDistance.setValue(maxDistance);
   }

   public double getMinDistance()
   {
      return minDistance.getValue();
   }

   public void setMinDistance(double minDistance)
   {
      this.minDistance.setValue(minDistance);
   }

   public DistanceType getDistanceType()
   {
      return distanceType.getValue();
   }

   public void setDistanceType(DistanceType distanceType)
   {
      this.distanceType.setValue(distanceType);
   }

   public double getTimeout()
   {
      return timeout.getValue();
   }
}