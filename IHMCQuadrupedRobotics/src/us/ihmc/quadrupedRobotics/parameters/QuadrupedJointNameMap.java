package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFHumanoidJointNameMap;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedJointNameMap extends SDFHumanoidJointNameMap 
{
   QuadrupedJointName getJointNameForSDFName(String name);

   String getJointBeforeFootName(RobotQuadrant robotQuadrant);

   String getLegJointName(RobotQuadrant robotQuadrant, LegJointName legJointName);
}