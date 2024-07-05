package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents a Vector3D that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalVector3D extends CRDTUnidirectionalMutableField<Vector3D>
{
   public CRDTUnidirectionalVector3D(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, Vector3D::new);
   }

   public Vector3DReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   public void toMessage(Vector3D message)
   {
      message.set(getValueReadOnly());
   }

   public void fromMessage(Vector3D message)
   {
      getValueInternal().set(message);
   }
}