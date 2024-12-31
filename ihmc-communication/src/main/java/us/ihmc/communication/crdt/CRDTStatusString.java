package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * An status String field. See {@link CRDTStatusImmutableField}.
 */
public class CRDTStatusString extends CRDTStatusImmutableField<String>
{
   public CRDTStatusString(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, String initialValue)
   {
      super(sideThatCanModify, crdtInfo, initialValue);
   }
}
