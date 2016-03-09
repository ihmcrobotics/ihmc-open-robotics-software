package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFQuadrupedJointNameMap;

public interface QuadrupedJointNameMap extends SDFQuadrupedJointNameMap 
{
   QuadrupedJointName getJointNameForSDFName(String name);
}