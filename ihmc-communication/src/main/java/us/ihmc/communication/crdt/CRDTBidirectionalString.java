package us.ihmc.communication.crdt;

/**
 * Represents a String that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalString extends CRDTBidirectionalImmutableField<String>
{
   public CRDTBidirectionalString(RequestConfirmFreezable requestConfirmFreezable, String initialValue)
   {
      super(requestConfirmFreezable, initialValue);
   }
}
