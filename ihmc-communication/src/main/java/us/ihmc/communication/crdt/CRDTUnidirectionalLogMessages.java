package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * An unidirectional CRDT for log messages.
 */
public class CRDTUnidirectionalLogMessages extends CRDTUnidirectionalImmutableField<String>
{
   public CRDTUnidirectionalLogMessages(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, String initialValue)
   {
      super(sideThatCanModify, crdtInfo, initialValue);
   }
}
