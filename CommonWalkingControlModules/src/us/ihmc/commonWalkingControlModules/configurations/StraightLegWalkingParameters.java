package us.ihmc.commonWalkingControlModules.configurations;

public class StraightLegWalkingParameters
{
   public boolean includeHipPitchPrivilegedConfiguration()
   {
      return false;
   }

   /**
    * This is the speed used to straighten the desire privileged configuration of the stance leg's knee.
    * @return rad/second for straightening
    */
   public double getSpeedForStanceLegStraightening()
   {
      return 0.25;
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

   public double getHipPrivilegedConfigurationGain()
   {
      return 40.0;
   }

   public double getHipPrivilegedVelocityGain()
   {
      return 6.0;
   }

   public double getHipPrivilegedWeight()
   {
      return 5.0;
   }

   public double getKneeStraightLegPrivilegedConfigurationGain()
   {
      return 40.0;
   }

   public double getKneeStraightLegPrivilegedVelocityGain()
   {
      return 6.0;
   }

   public double getKneeStraightLegPrivilegedWeight()
   {
      return 5.0;
   }

   public double getKneeBentLegPrivilegedConfigurationGain()
   {
      return 40.0;
   }

   public double getKneeBentLegPrivilegedVelocityGain()
   {
      return 6.0;
   }

   public double getKneeBentLegPrivilegedWeight()
   {
      return 5.0;
   }

   public double getPrivilegedMaxVelocity()
   {
      return 2.0;
   }

   public double getPrivilegedMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }
}
