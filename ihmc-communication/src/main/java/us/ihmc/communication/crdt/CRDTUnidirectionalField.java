package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalField<T>
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final CRDTInfo crdtInfo;

   private T value;

   public CRDTUnidirectionalField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, T initialValue)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.crdtInfo = crdtInfo;

      value = initialValue;
   }

   public T getValue()
   {
      return value;
   }

   public void setValue(T value)
   {
      if (sideThatCanModify != crdtInfo.getActorDesignation())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));

      this.value = value;
   }

   public void fromMessage(T value)
   {
      if (sideThatCanModify == crdtInfo.getActorDesignation() && !value.equals(this.value))
         LogTools.error("Received unexpected value. Expected: %s  Received: %s", value.toString(), this.value.toString());
       else
          this.value = value;
   }

   public T toMessage()
   {
      return value;
   }
}
