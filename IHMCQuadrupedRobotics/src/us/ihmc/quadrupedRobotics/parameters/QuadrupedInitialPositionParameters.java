package us.ihmc.quadrupedRobotics.parameters;

public interface QuadrupedInitialPositionParameters
{
   /**
    * @return the height at which the root frame stands off the ground.
    */
   public double getInitialHeight();

   /**
    * Maps joint names to initial positions.
    * 
    * @param joint
    *           the joint name to lookup.
    * @return the initial angle in radians.
    */
   public double getInitialPosition(QuadrupedJointName joint);
}
