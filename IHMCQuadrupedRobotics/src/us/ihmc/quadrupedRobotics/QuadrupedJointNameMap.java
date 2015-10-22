package us.ihmc.quadrupedRobotics;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedJointNameMap extends SDFJointNameMap 
{

   String getJointBeforeFootName(RobotQuadrant robotQuadrant);

   String getLegJointName(RobotQuadrant robotQuadrant, LegJointName legJointName);

}