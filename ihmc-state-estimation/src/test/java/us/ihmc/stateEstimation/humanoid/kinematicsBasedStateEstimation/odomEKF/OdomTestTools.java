package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.Random;

public class OdomTestTools
{
   public static void setRandomSensed(Random random, SensedVariables sensedVariablesToPack)
   {
      sensedVariablesToPack.gyroMeasurement.set(EuclidCoreRandomTools.nextVector3D(random, 5.0));
      sensedVariablesToPack.accelMeasurement.set(EuclidCoreRandomTools.nextVector3D(random, 5.0));
      sensedVariablesToPack.positionMeasurement.set(EuclidCoreRandomTools.nextVector3D(random, 5.0));
      sensedVariablesToPack.orientationMeasurement.set(EuclidCoreRandomTools.nextQuaternion(random, 5.0));
      sensedVariablesToPack.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 5.0));
   }

   public static DMatrixRMaj setRandomState(Random random, SensedVariables sensedVariables, StateVariables stateVariablesToPack)
   {
      stateVariablesToPack.translation.set(EuclidCoreRandomTools.nextPoint3D(random, 10.0));
      stateVariablesToPack.linearVelocity.set(EuclidCoreRandomTools.nextVector3D(random, 10.0));
      stateVariablesToPack.orientation.set(EuclidCoreRandomTools.nextQuaternion(random));
      stateVariablesToPack.accelBias.set(EuclidCoreRandomTools.nextVector3D(random, 5.0));
      stateVariablesToPack.gyroBias.set(EuclidCoreRandomTools.nextVector3D(random, 5.0));

      if (OdometryKalmanFilter.includeBias)
      {
         stateVariablesToPack.unbiasedAccel.sub(sensedVariables.accelMeasurement, stateVariablesToPack.accelBias);
         stateVariablesToPack.unbiasedGyro.sub(sensedVariables.gyroMeasurement, stateVariablesToPack.gyroBias);
      }
      else
      {
         stateVariablesToPack.unbiasedAccel.set(sensedVariables.accelMeasurement);
         stateVariablesToPack.unbiasedGyro.set(sensedVariables.gyroMeasurement);
      }

      DMatrixRMaj stateVector = new DMatrixRMaj(OdometryIndexHelper.stateSizePerLink, 1);
      stateVariablesToPack.translation.get(stateVector);
      stateVariablesToPack.linearVelocity.get(3, stateVector);
      stateVariablesToPack.orientation.get(6, stateVector);
      stateVariablesToPack.accelBias.get(10, stateVector);
      stateVariablesToPack.gyroBias.get(13, stateVector);

      return stateVector;
   }
}
