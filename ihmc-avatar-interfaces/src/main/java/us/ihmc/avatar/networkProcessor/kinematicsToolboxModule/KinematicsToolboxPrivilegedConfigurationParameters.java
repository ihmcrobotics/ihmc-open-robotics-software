package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;

public class KinematicsToolboxPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters {
	public KinematicsToolboxPrivilegedConfigurationParameters()
	   {
	   }

	   /**
	    * To compute the nullspace of the high-level control objectives, a matrix inversion has to be
	    * calculated. To improve stability, a damped-least square pseudo-inverse is used, which in turn
	    * uses this alpha parameter.
	    * <p>
	    * It should preferably be close to zero.
	    * </p>
	    * 
	    * @return the parameter used for computing the damped-least squares pseudo-inverse.
	    */
	   public double getNullspaceProjectionAlpha()
	   {
	      return 0.005;
	   }

	   /**
	    * <p>
	    * Returns the desired default configuration gain for the joint privileged configuration handler.
	    * </p>
	    * <p>
	    * This gain is used to track the privileged configuration when no other gain is provided.
	    * </p>
	    * 
	    * @return configuration gain
	    */
	   public double getDefaultConfigurationGain()
	   {
	      return 40.0;
	   }

	   /**
	    * <p>
	    * Returns the desired default velocity gain for the joint privileged configuration handler.
	    * </p>
	    * <p>
	    * This gain is used to damp the velocities when no other velocity gain is provided.
	    * </p>
	    * <p>
	    * Note that when computing privileged velocities, this value is not used.
	    * </p>
	    * 
	    * @return velocity gain
	    */
	   public double getDefaultVelocityGain()
	   {
	      return 6.0;
	   }

	   /**
	    * <p>
	    * Returns the default maximum velocity for the joint privileged configuration handler.
	    * </p>
	    * <p>
	    * This limits the maximum velocity when computing privileged velocity commands that will be
	    * allowed to attempt to reach the privileged configuration.
	    * </p>
	    * 
	    * @return max joint velocity (rad / s)
	    */
	   public double getDefaultMaxVelocity()
	   {
	      return 2.0;
	   }

	   /**
	    * <p>
	    * Returns the default maximum acceleration for the joint privileged configuration handler.
	    * </p>
	    * <p>
	    * This limits the maximum acceleration when computing privileged acceleration commands that will
	    * be allowed to attempt to reach the privileged configuration.
	    * </p>
	    * 
	    * @return max joint acceleration (rad / s^2)
	    */
	   public double getDefaultMaxAcceleration()
	   {
	      return Double.POSITIVE_INFINITY;
	   }

	   /**
	    * <p>
	    * Returns the default weight for the joint privileged configuration handler.
	    * </p>
	    * <p>
	    * This is the weight placed on the privileged configuration command in the nullspace of the task
	    * Jacobian
	    * </p>
	    * 
	    * @return privileged command weight
	    */
	   public double getDefaultWeight()
	   {
	      return 5.0;
	   }
}
