package us.ihmc.communication.crdt;

import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

public class CRDTUnidirectionalSpatialVector extends CRDTUnidirectionalMutableField<SpatialVector>
{
   public CRDTUnidirectionalSpatialVector(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, SpatialVector::new);
   }

   public SpatialVectorReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   public void toMessage(SpatialVectorMessage message)
   {
      MessageTools.toMessage(getValueReadOnly(), message);
   }

   public void fromMessage(SpatialVectorMessage message)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         MessageTools.fromMessage(message, getValueInternal());
      }
   }
}