package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface SDFQuadrupedJointNameMap extends SDFJointNameMap
{
   public String getLegJointName(RobotQuadrant robotQuadrant, LegJointName legJointName);

   public String getJointBeforeFootName(RobotQuadrant robotQuadrant);

   QuadrupedJointName getJointNameForSDFName(String name);

   String getSDFNameForJointName(QuadrupedJointName name);
}
