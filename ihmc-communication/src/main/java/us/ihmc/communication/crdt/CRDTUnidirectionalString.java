package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * An unidirectional String field. See {@link CRDTUnidirectionalImmutableField}.
 */
public class CRDTUnidirectionalString extends CRDTUnidirectionalImmutableField<String>
{
   public CRDTUnidirectionalString(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, String initialValue)
   {
      super(sideThatCanModify, crdtInfo, initialValue);
   }
}
