package us.ihmc.communication.crdt;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Represents a Point3D that that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalPoint3D extends CRDTBidirectionalMutableField<Point3D>
{
   public CRDTBidirectionalPoint3D(RequestConfirmFreezable requestConfirmFreezable)
   {
      super(requestConfirmFreezable, new Point3D());
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
      if (!isFrozen())
      {
         getValueInternal().set(message);
      }
   }
}