package us.ihmc.commonWalkingControlModules.configurations;

import gnu.trove.map.hash.TObjectDoubleHashMap;

/**
 * Created by Christian DeBuys on 2023-09-26
 */
public abstract class JointOfflineParameters
{
   public abstract TObjectDoubleHashMap<String> getDampingCoefficentPerJoint();
}
