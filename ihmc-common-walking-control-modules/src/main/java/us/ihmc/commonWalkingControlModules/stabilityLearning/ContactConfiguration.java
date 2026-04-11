package us.ihmc.commonWalkingControlModules.stabilityLearning;

public enum ContactConfiguration
{
   SINGLE_SUPPORT_SAME_SIDE_(false, true, false),
   SINGLE_SUPPORT_OPPOSITE_SIDE_(false, false, true),
   SINGLE_SUPPORT_DUAL_HAND_(true, true, false),

   DOUBLE_SUPPORT_SINGLE_HAND_(false, true, true),
   DOUBLE_SUPPORT_DUAL_HAND_(true, true, true);

   private final boolean isOppositeHandInContact;
   private final boolean isSameFootInContact;
   private final boolean isOppositeFootInContact;

   ContactConfiguration(boolean isOppositeHandInContact, boolean isSameFootInContact, boolean isOppositeFootInContact)
   {
      this.isOppositeHandInContact = isOppositeHandInContact;
      this.isSameFootInContact = isSameFootInContact;
      this.isOppositeFootInContact = isOppositeFootInContact;
   }

   public boolean isOppositeHandInContact()
   {
      return isOppositeHandInContact;
   }

   public boolean isSameFootInContact()
   {
      return isSameFootInContact;
   }

   public boolean isOppositeFootInContact()
   {
      return isOppositeFootInContact;
   }

   public boolean isDoubleSupport()
   {
      return isSameFootInContact && isOppositeFootInContact;
   }

   public int getNetworkInputSize()
   {
      int numContactingHands = isOppositeHandInContact ? 2 : 1;
      return 6 * numContactingHands + (isDoubleSupport() ? 3 : 2);
   }
}
