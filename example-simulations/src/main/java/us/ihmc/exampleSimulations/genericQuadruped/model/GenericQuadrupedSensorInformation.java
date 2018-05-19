package us.ihmc.exampleSimulations.genericQuadruped.model;

import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;

public class GenericQuadrupedSensorInformation implements QuadrupedSensorInformation
{
   private static final String[] imuNames = new String[] {"body_imu"};
   
   @Override
   public String[] getImuNames()
   {
      return imuNames;
   }
}
