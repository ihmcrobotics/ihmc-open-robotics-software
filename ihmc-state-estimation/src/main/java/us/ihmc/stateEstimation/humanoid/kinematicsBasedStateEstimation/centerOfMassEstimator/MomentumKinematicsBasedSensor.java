package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.addEquals;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.setDiagonalElements;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MomentumKinematicsBasedSensor extends Sensor
{
   private final int posCoM;
   private final int posLinMom;
   private final int posAngMom;
   private final int size;
   private final Point3DReadOnly measuredCoMPosition;
   private final Vector3DReadOnly measuredLinearMomentum;
   private final Vector3DReadOnly measuredAngularMomentum;

   private final YoVector3D centerOfMassPositionResidual;
   private final YoVector3D linearMomentumResidual;
   private final YoVector3D angularMomentumResidual;

   private final double sqrtHz;
   private final DoubleProvider centerOfMassVariance;
   private final DoubleProvider linearMomentumVariance;
   private final DoubleProvider angularMomentumVariance;

   private final MomentumEstimatorIndexProvider stateIndexProvider;

   private final DMatrixRMaj stateVector;

   public MomentumKinematicsBasedSensor(Point3DReadOnly measuredCoMPosition,
                                        Vector3DReadOnly measuredLinearMomentum,
                                        Vector3DReadOnly measuredAngularMomentum,
                                        MomentumEstimatorIndexProvider stateIndexProvider,
                                        double dt,
                                        YoRegistry registry)
   {
      this.measuredCoMPosition = measuredCoMPosition;
      this.measuredLinearMomentum = measuredLinearMomentum;
      this.measuredAngularMomentum = measuredAngularMomentum;
      this.stateIndexProvider = stateIndexProvider;

      int currentIndex = 0;
      posCoM = currentIndex;
      currentIndex += 3;
      if (measuredLinearMomentum != null)
      {
         posLinMom = currentIndex;
         currentIndex += 3;
      }
      else
      {
         posLinMom = -1;
      }
      posAngMom = currentIndex;
      size = currentIndex + 3;

      sqrtHz = 1.0 / Math.sqrt(dt);
      centerOfMassVariance = FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, MathTools.square(0.001));
      centerOfMassPositionResidual = new YoVector3D("centerOfMassPositionResidual", registry);

      if (measuredLinearMomentum != null)
      {
         linearMomentumVariance = FilterTools.findOrCreate("kinematicsBasedLinearMomentumVariance", registry, MathTools.square(1.0));
         linearMomentumResidual = new YoVector3D("linearMomentumResidual", registry);
      }
      else
      {
         linearMomentumVariance = null;
         linearMomentumResidual = null;
      }

      angularMomentumVariance = FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance", registry, MathTools.square(1.0));
      angularMomentumResidual = new YoVector3D("angularMomentumResidual", registry);

      stateVector = new DMatrixRMaj(size, 1);
   }

   @Override
   public String getName()
   {
      return "ExtendedMomentumSensor";
   }

   @Override
   public int getMeasurementSize()
   {
      return size;
   }

   /**
    * <pre>
    * / 1 0 0  0 0 0  0 0 0  1 0  0 0 0 \
    * | 0 1 0  0 0 0  0 0 0  0 1  0 0 0 |
    * | 0 0 1  0 0 0  0 0 0  0 0  0 0 0 |
    * | 0 0 0  1 0 0  0 0 0  0 0  1 0 0 |
    * | 0 0 0  0 1 0  0 0 0  0 0  0 1 0 |
    * | 0 0 0  0 0 1  0 0 0  0 0  0 0 1 |
    * | 0 0 0  0 0 0  1 0 0  0 0  0 0 0 |
    * | 0 0 0  0 0 0  0 1 0  0 0  0 0 0 |
    * \ 0 0 0  0 0 0  0 0 1  0 0  0 0 0 /
    * </pre>
    */
   @Override
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      jacobianToPack.reshape(size, robotState.getSize());
      jacobianToPack.zero();

      setDiagonalElements(posCoM, stateIndexProvider.getCoMPosition(), 3, 1.0, jacobianToPack);
      if (measuredLinearMomentum != null)
         setDiagonalElements(posLinMom, stateIndexProvider.getLinearMomentum(), 3, 1.0, jacobianToPack);
      setDiagonalElements(posAngMom, stateIndexProvider.getAngularMomentum(), 3, 1.0, jacobianToPack);
      if (stateIndexProvider.hasCoMPositionOffset())
         setDiagonalElements(posCoM, stateIndexProvider.getCoMPositionOffset(), 2, 1.0, jacobianToPack);
      if (measuredLinearMomentum != null && stateIndexProvider.hasLinearMomentumOffset())
         setDiagonalElements(posLinMom, stateIndexProvider.getLinearMomentumOffset(), 3, 1.0, jacobianToPack);
   }

   /**
    * <pre>
    * / x_com^meas    - x_com^pred    \
    * | y_com^meas    - y_com^pred    |
    * | z_com^meas    - z_com^pred    |
    * | x_angMom^meas - x_angMom^pred |
    * | y_angMom^meas - y_angMom^pred |
    * \ z_angMom^meas - z_angMom^pred /
    * </pre>
    */
   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      robotState.getStateVector(stateVector);
      residualToPack.reshape(getMeasurementSize(), 1);

      centerOfMassPositionResidual.set(stateIndexProvider.getCoMPosition(), stateVector);
      if (measuredLinearMomentum != null)
         linearMomentumResidual.set(stateIndexProvider.getLinearMomentum(), stateVector);
      angularMomentumResidual.set(stateIndexProvider.getAngularMomentum(), stateVector);

      if (stateIndexProvider.hasCoMPositionOffset())
         addEquals(centerOfMassPositionResidual, stateVector, stateIndexProvider.getCoMPositionOffset(), 0);

      if (measuredLinearMomentum != null && stateIndexProvider.hasLinearMomentumOffset())
         addEquals(linearMomentumResidual, stateVector, stateIndexProvider.getLinearMomentumOffset(), 0);

      centerOfMassPositionResidual.sub(measuredCoMPosition, centerOfMassPositionResidual);
      if (measuredLinearMomentum != null)
         linearMomentumResidual.sub(measuredLinearMomentum, linearMomentumResidual);
      angularMomentumResidual.sub(measuredAngularMomentum, angularMomentumResidual);

      centerOfMassPositionResidual.get(posCoM, residualToPack);
      if (measuredLinearMomentum != null)
         linearMomentumResidual.get(posLinMom, residualToPack);
      angularMomentumResidual.get(posAngMom, residualToPack);
   }

   @Override
   public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
   {
      noiseCovarianceToPack.reshape(getMeasurementSize(), getMeasurementSize());
      noiseCovarianceToPack.zero();

      setDiagonalElements(posCoM, posCoM, 3, centerOfMassVariance.getValue() * sqrtHz, noiseCovarianceToPack);
      if (measuredLinearMomentum != null)
         setDiagonalElements(posLinMom, posLinMom, 3, linearMomentumVariance.getValue() * sqrtHz, noiseCovarianceToPack);
      setDiagonalElements(posAngMom, posAngMom, 3, angularMomentumVariance.getValue() * sqrtHz, noiseCovarianceToPack);
   }
}