package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.measurementAccelIndex;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.measurementRelativeTranslationIndex;

public class ObservationModel
{
   // State providers
   private final SensedVariables footMeasurements;

   public ObservationModel(SensedVariables footMeasurements)
   {
      this.footMeasurements = footMeasurements;
   }

   public void get(int startRow, DMatrixRMaj observation)
   {
      footMeasurements.positionMeasurement.get(startRow + measurementRelativeTranslationIndex, observation);
      footMeasurements.accelMeasurement.get(startRow + measurementAccelIndex, observation);
   }
}
