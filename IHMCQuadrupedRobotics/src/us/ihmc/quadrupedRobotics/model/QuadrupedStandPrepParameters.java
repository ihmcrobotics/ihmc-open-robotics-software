package us.ihmc.quadrupedRobotics.model;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

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
   
   public double getInitialJointPosition(RobotQuadrant robotQuadrant, LegJointName legJointName);
   
   public Point3d getInitialCOMPosition();
   
   public Point3d getInitialBodyPosition();
}
