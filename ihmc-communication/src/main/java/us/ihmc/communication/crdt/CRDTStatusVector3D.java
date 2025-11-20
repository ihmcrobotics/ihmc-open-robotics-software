package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents a Vector3D that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTStatusVector3D extends CRDTStatusMutableField<Vector3D>
{
   public CRDTStatusVector3D(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, Vector3D::new);
   }

   public Vector3DReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   /** Prefer this method in the case you need to call it every tick, as it no-ops in the case the value is the same. */
   public void setValue(Vector3DReadOnly value, double epsilon)
   {
      if (!(getValueInternal().epsilonEquals(value, epsilon)))
      {
         accessValue().set(value);
      }
   }

   public void toMessage(Vector3D message)
   {
      message.set(getValueReadOnly());
   }

   public void fromMessage(Vector3D message)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().set(message);
      }
   }
}