package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;

public class LegConfigurationParameters
{
   /**
    * Determines whether or not to attempt to use straight legs when indirectly controlling the center of mass
    * height using the nullspace in the full task Jacobian.
    * This will not do anything noticeable unless {@link WalkingControllerParameters#enableHeightFeedbackControl()}
    * returns true, as that indicates whether or not to use the pelvis to control the center of mass height.
    *
    * @return boolean (true = try and straighten, false = do not try and straighten)
    */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }


   /**
    * This is the weight placed on the knee privileged joint accelerations or velocities when the leg is in
    * the bent leg state in the optimization. For a typical humanoid, these joints are the hip pitch and
    * ankle pitch.
    */
   public double getLegPrivilegedLowWeight()
   {
      return 5.0;
   }

   /**
    * These are the configuration gain used to control the knee privileged joint accelerations or privileged joint velocities
    * when the leg is in the straight leg state or straightening state.
    * It contains the gains used by the {@link us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule} to determine either
    * the privileged acceleration or the privileged velocity to project into the nullspace of the full task Jacobian.
    *
    * @return privileged configuration gain.
    */
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(40.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   /**
    * The maximum privileged acceleration to be allowed after feedback as an objective in the optimization, as
    * calculated by the {@link JointPrivilegedConfigurationHandler}.
    * This value is used for all the leg pitch joints, which, for a standard humanoid, is the hip pitch, knee pitch
    * and ankle pitch.
    *
    * @return max privileged acceleration in radians per second.
    */
   public double getPrivilegedMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }
}
