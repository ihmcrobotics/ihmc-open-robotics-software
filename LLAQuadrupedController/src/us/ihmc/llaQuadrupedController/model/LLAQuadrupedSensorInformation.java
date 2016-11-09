package us.ihmc.llaQuadrupedController.model;

import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;

public class LLAQuadrupedSensorInformation implements QuadrupedSensorInformation
{
   private static final String[] imuNames = new String[] {"body_imu"};
   
   @Override
   public String[] getImuNames()
   {
      return imuNames;
   }
}
