package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalDouble
{
   private final RequestConfirmFreezable requestConfirmFreezable;

   private double value;

   public CRDTBidirectionalDouble(RequestConfirmFreezable requestConfirmFreezable, double initialValue)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;

      value = initialValue;
   }

   /** For use when the initial value requires special calulation. */
   public void setInitialValue(double initialValue)
   {
      value = initialValue;
   }

   public double getValue()
   {
      return value;
   }

   public void setValue(double value)
   {
      if (this.value != value) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         requestConfirmFreezable.freeze();
      }
   }

   public double toMessage()
   {
      return value;
   }

   public void fromMessage(double value)
   {
      if (!requestConfirmFreezable.isFrozen())
      {
         this.value = value;
      }
   }
}
