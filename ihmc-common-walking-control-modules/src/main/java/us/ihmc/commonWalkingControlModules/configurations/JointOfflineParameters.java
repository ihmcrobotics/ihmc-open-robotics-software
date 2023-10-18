package us.ihmc.commonWalkingControlModules.configurations;

import gnu.trove.map.hash.TObjectDoubleHashMap;

/**
 * Implementations of this class should contain a method to return the backdriving characteristics of the joints. The friction coefficient can be approximated
 * from the torque-speed curve, either as a constant or as a funcntion of angular velocity.
 * <p>
 * Created by Christian DeBuys on 2023-09-26
 */
public abstract class JointOfflineParameters
{
   public abstract TObjectDoubleHashMap<String> getDampingCoefficentPerJoint();
}
