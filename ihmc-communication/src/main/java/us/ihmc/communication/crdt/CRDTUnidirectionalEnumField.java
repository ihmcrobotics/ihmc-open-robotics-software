package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents an enum field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalEnumField<T extends Enum<T>> extends CRDTUnidirectionalImmutableField<T>
{
   public CRDTUnidirectionalEnumField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, T initialValue)
   {
      super(sideThatCanModify, crdtInfo, initialValue);
   }
}
