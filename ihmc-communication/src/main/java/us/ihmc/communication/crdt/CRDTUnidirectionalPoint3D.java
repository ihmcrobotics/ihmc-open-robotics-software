package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Represents a Point3D that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalPoint3D extends CRDTUnidirectionalMutableField<Point3D>
{
   public CRDTUnidirectionalPoint3D(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      super(sideThatCanModify, crdtInfo, Point3D::new);
   }

   public Point3DReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   public void toMessage(Point3D message)
   {
      message.set(getValueReadOnly());
   }

   public void fromMessage(Point3D message)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         getValueInternal().set(message);
      }
   }
}