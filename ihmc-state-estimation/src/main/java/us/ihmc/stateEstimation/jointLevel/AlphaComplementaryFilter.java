package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBasedJointStateEstimator;

import java.util.List;

public class AlphaComplementaryFilter
{
   private final List<IMUBasedJointStateEstimator> IMUEstimators;
   private final IMUBasedJointStateEstimator localEstimator;
}
