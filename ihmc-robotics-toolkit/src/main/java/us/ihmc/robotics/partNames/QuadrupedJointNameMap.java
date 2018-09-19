package us.ihmc.robotics.partNames;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedJointNameMap extends LeggedJointNameMap<RobotQuadrant>
{
   QuadrupedJointName getJointNameForSDFName(String name);

   String getSDFNameForJointName(QuadrupedJointName name);

   String getBodyName();
   
   String getLegJointName(RobotQuadrant robotQuadrant, LegJointName legJointName);

   @Override
   default RobotQuadrant[] getRobotSegments()
   {
      return RobotQuadrant.values;
   }

   @Override
   default String getRootBodyName()
   {
      return getBodyName();
   }
}
