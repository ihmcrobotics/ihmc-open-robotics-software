package us.ihmc.quadrupedRobotics.model;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.partNames.QuadrupedJointName;

public interface QuadrupedStandPrepParameters
{
   /**
    * Maps joint names to initial positions.
    *
    * @param joint
    *           the joint name to lookup.
    * @return the initial angle in radians.
    */
   public double getInitialJointPosition(QuadrupedJointName joint);
   
   public Point3d getInitialCOMPosition();
   
   public Point3d getInitialBodyPosition();
}
