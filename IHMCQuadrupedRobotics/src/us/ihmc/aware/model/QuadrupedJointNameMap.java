package us.ihmc.aware.model;

import us.ihmc.SdfLoader.SDFQuadrupedJointNameMap;
import us.ihmc.aware.model.QuadrupedJointName;

public interface QuadrupedJointNameMap extends SDFQuadrupedJointNameMap 
{
   QuadrupedJointName getJointNameForSDFName(String name);
   
   String getSDFNameForJointName(QuadrupedJointName name);
}