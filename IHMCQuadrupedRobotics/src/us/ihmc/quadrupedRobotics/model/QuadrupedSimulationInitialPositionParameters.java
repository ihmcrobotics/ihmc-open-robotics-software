package us.ihmc.quadrupedRobotics.model;

import us.ihmc.euclid.tuple3D.Point3D;

import us.ihmc.robotics.partNames.QuadrupedJointName;

public interface QuadrupedSimulationInitialPositionParameters
{
   /**
    * Maps joint names to initial positions.
    *
    * @param joint
    *           the joint name to lookup.
    * @return the initial angle in radians.
    */
   public double getInitialJointPosition(QuadrupedJointName joint);

   public Point3D getInitialBodyPosition();
}
