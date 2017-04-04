package us.ihmc.commonWalkingControlModules.configurations;

public class StraightLegWalkingParameters
{
   /**
    * This is the duration used to straighten the desire privileged configuration of the stance leg's knee.
    * @return time in seconds for straightening
    */
   public double getDurationForStanceLegStraightening()
   {
      return 1.3;
   }

   /**
    * Angle used by the privileged configuration that is defined as straight for the knees.
    * @return angle in radians
    */
   public double getStraightKneeAngle()
   {
      return 0.2;
   }

   /**
    * Returns a percent of the swing state to switch the privileged configuration to having straight knees
    * @return ratio of swing state (0.0 to 1.0)
    */
   public double getPercentOfSwingToStraightenLeg()
   {
      return 0.8;
   }

   /**
    * Returns a percent of the transfer state to switch the privileged configuration to having bent knees
    * @return ratio of transfer state (0.0 to 1.0)
    */
   public double getPercentOfTransferToCollapseLeg()
   {
      return 0.9;
   }

   /**
    * Determines whether or not to attempt to use straight legs when controlling the height in the nullspace.
    * This will not do anything noticeable unless {@link WalkingControllerParameters#controlHeightWithMomentum()} returns true.
    * @return boolean (true = try and straighten, false = do not try and straighten)
    */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }
}
