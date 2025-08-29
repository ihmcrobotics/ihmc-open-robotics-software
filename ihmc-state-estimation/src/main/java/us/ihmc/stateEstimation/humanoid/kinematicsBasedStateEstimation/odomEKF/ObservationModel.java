package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryKalmanFilter.MeasuredVariables;

public class ObservationModel
{
   // State providers
   private final MeasuredVariables footMeasurements;

   public ObservationModel(MeasuredVariables footMeasurements)
   {
      this.footMeasurements = footMeasurements;
   }

   public void get(int startRow, DMatrixRMaj observation)
   {
      footMeasurements.positionMeasurement.get(startRow, observation);
      footMeasurements.accelMeasurement.get(startRow + 6, observation);
   }
}
