package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.addEquals;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.computeCoP;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.setDiagonalElements;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.Estimator;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MomentumKinematicsBasedEKFSensor extends Sensor
{
   private final int posCoM;
   private final int posLinMom;
   private final int posAngMom;
   private final int posCoP;
   private final int size;
   private final Point3DReadOnly measuredCoMPosition;
   private final Vector3DReadOnly measuredLinearMomentum;
   private final Vector3DReadOnly measuredAngularMomentum;
   private final Point2DReadOnly measuredCenterOfPressure;

   private final YoVector3D centerOfMassPositionResidual;
   private final YoVector3D linearMomentumResidual;
   private final YoVector3D angularMomentumResidual;
   private final YoVector2D centerOfPressureResidual;

   private final double sqrtHz;
   private final DoubleProvider centerOfMassVariance;
   private final DoubleProvider linearMomentumVariance;
   private final DoubleProvider angularMomentumVariance;
   private final DoubleProvider centerOfPressureVariance;

   private final MomentumEKFEstimatorIndexProvider stateIndexProvider;
   private final WrenchBasedMomentumRateCalculator momentumRateCalculator;

   private final DMatrixRMaj stateVector;

   public static MomentumKinematicsBasedEKFSensor createEstimator(Estimator estimator,
                                                                  Point3DReadOnly measuredCoMPosition,
                                                                  Vector3DReadOnly measuredLinearMomentum,
                                                                  Vector3DReadOnly measuredAngularMomentum,
                                                                  Point2DReadOnly measuredCenterOfPressure,
                                                                  MomentumEKFEstimatorIndexProvider stateIndexProvider,
                                                                  WrenchBasedMomentumRateCalculator momentumRateCalculator,
                                                                  double dt,
                                                                  YoRegistry registry)
   {
      switch (estimator)
      {
         case MomentumEstimator:
            return createMomentumEstimatorSensor(measuredCoMPosition, measuredAngularMomentum, stateIndexProvider, dt, registry);
         case OffsetEstimator:
            return createOffsetEstimatorSensor(measuredCoMPosition, measuredLinearMomentum, measuredAngularMomentum, stateIndexProvider, dt, registry);
         case CoPBasedOffsetEstimator:
            return createCoPBasedOffsetEstimatorSensor(measuredCoMPosition,
                                                       measuredLinearMomentum,
                                                       measuredAngularMomentum,
                                                       measuredCenterOfPressure,
                                                       stateIndexProvider,
                                                       momentumRateCalculator,
                                                       dt,
                                                       registry);
         default:
            throw new IllegalArgumentException();
      }
   }

   public static MomentumKinematicsBasedEKFSensor createMomentumEstimatorSensor(Point3DReadOnly measuredCoMPosition,
                                                                                Vector3DReadOnly measuredAngularMomentum,
                                                                                MomentumEKFEstimatorIndexProvider stateIndexProvider,
                                                                                double dt,
                                                                                YoRegistry registry)
   {
      return new MomentumKinematicsBasedEKFSensor(measuredCoMPosition,
                                                  null,
                                                  measuredAngularMomentum,
                                                  null,
                                                  FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, MathTools.square(0.0001 / 32.6228)),
                                                  null,
                                                  FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance", registry, MathTools.square(0.1 / 32.6228)),
                                                  null,
                                                  stateIndexProvider,
                                                  null,
                                                  dt,
                                                  registry);
   }

   public static MomentumKinematicsBasedEKFSensor createOffsetEstimatorSensor(Point3DReadOnly measuredCoMPosition,
                                                                              Vector3DReadOnly measuredLinearMomentum,
                                                                              Vector3DReadOnly measuredAngularMomentum,
                                                                              MomentumEKFEstimatorIndexProvider stateIndexProvider,
                                                                              double dt,
                                                                              YoRegistry registry)
   {
      return new MomentumKinematicsBasedEKFSensor(measuredCoMPosition,
                                                  measuredLinearMomentum,
                                                  measuredAngularMomentum,
                                                  null,
                                                  FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, MathTools.square(0.001 / 32.6228)),
                                                  FilterTools.findOrCreate("kinematicsBasedLinearMomentumVariance", registry, MathTools.square(1.0 / 32.6228)),
                                                  FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance", registry, MathTools.square(1.0 / 32.6228)),
                                                  null,
                                                  stateIndexProvider,
                                                  null,
                                                  dt,
                                                  registry);
   }

   public static MomentumKinematicsBasedEKFSensor createCoPBasedOffsetEstimatorSensor(Point3DReadOnly measuredCoMPosition,
                                                                                      Vector3DReadOnly measuredLinearMomentum,
                                                                                      Vector3DReadOnly measuredAngularMomentum,
                                                                                      Point2DReadOnly measuredCenterOfPressure,
                                                                                      MomentumEKFEstimatorIndexProvider stateIndexProvider,
                                                                                      WrenchBasedMomentumRateCalculator momentumRateCalculator,
                                                                                      double dt,
                                                                                      YoRegistry registry)
   {
      return new MomentumKinematicsBasedEKFSensor(measuredCoMPosition,
                                                  measuredLinearMomentum,
                                                  measuredAngularMomentum,
                                                  measuredCenterOfPressure,
                                                  FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, MathTools.square(0.001 / 32.6228)),
                                                  FilterTools.findOrCreate("kinematicsBasedLinearMomentumVariance", registry, MathTools.square(1.0 / 32.6228)),
                                                  FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance",
                                                                           registry,
                                                                           MathTools.square(10.0 / 32.6228)),
                                                  FilterTools.findOrCreate("measuredCenterOfPressureVariance", registry, MathTools.square(1.0 / 32.6228)),
                                                  stateIndexProvider,
                                                  momentumRateCalculator,
                                                  dt,
                                                  registry);
   }

   public MomentumKinematicsBasedEKFSensor(Point3DReadOnly measuredCoMPosition,
                                           Vector3DReadOnly measuredLinearMomentum,
                                           Vector3DReadOnly measuredAngularMomentum,
                                           Point2DReadOnly measuredCenterOfPressure,
                                           double centerOfMassVariance,
                                           double linearMomentumVariance,
                                           double angularMomentumVariance,
                                           double centerOfPressureVariance,
                                           MomentumEKFEstimatorIndexProvider stateIndexProvider,
                                           WrenchBasedMomentumRateCalculator momentumRateCalculator,
                                           double dt,
                                           YoRegistry registry)
   {
      this(measuredCoMPosition, measuredLinearMomentum, measuredAngularMomentum, measuredCenterOfPressure,
           FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, centerOfMassVariance),
           FilterTools.findOrCreate("kinematicsBasedLinearMomentumVariance", registry, linearMomentumVariance),
           FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance", registry, angularMomentumVariance),
           FilterTools.findOrCreate("measuredCenterOfPressureVariance", registry, centerOfPressureVariance), stateIndexProvider, momentumRateCalculator, dt,
           registry);
   }

   public MomentumKinematicsBasedEKFSensor(Point3DReadOnly measuredCoMPosition,
                                           Vector3DReadOnly measuredLinearMomentum,
                                           Vector3DReadOnly measuredAngularMomentum,
                                           Point2DReadOnly measuredCenterOfPressure,
                                           DoubleProvider centerOfMassVariance,
                                           DoubleProvider linearMomentumVariance,
                                           DoubleProvider angularMomentumVariance,
                                           DoubleProvider centerOfPressureVariance,
                                           MomentumEKFEstimatorIndexProvider stateIndexProvider,
                                           WrenchBasedMomentumRateCalculator momentumRateCalculator,
                                           double dt,
                                           YoRegistry registry)
   {
      this.measuredCoMPosition = measuredCoMPosition;
      this.measuredLinearMomentum = measuredLinearMomentum;
      this.measuredAngularMomentum = measuredAngularMomentum;
      this.measuredCenterOfPressure = measuredCenterOfPressure;
      this.centerOfMassVariance = centerOfMassVariance;
      this.linearMomentumVariance = linearMomentumVariance;
      this.angularMomentumVariance = angularMomentumVariance;
      this.centerOfPressureVariance = centerOfPressureVariance;
      this.stateIndexProvider = stateIndexProvider;
      this.momentumRateCalculator = momentumRateCalculator;

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
      currentIndex += 3;
      if (measuredCenterOfPressure != null)
      {
         posCoP = currentIndex;
         currentIndex += 2;
      }
      else
      {
         posCoP = -1;
      }
      size = currentIndex;

      sqrtHz = 1.0 / Math.sqrt(dt);
      centerOfMassPositionResidual = new YoVector3D("centerOfMassPositionResidual", registry);

      if (measuredLinearMomentum != null)
      {
         linearMomentumResidual = new YoVector3D("linearMomentumResidual", registry);
      }
      else
      {
         linearMomentumVariance = null;
         linearMomentumResidual = null;
      }

      angularMomentumResidual = new YoVector3D("angularMomentumResidual", registry);

      if (measuredCenterOfPressure != null)
      {
         centerOfPressureResidual = new YoVector2D("centerOfPressureResidual", registry);
      }
      else
      {
         centerOfPressureResidual = null;
      }

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

      if (measuredCenterOfPressure != null)
      {
         setDiagonalElements(posCoP, posCoM, 2, 2.0, jacobianToPack);
         YoFrameVector3D force = momentumRateCalculator.getTotalSpatialForceAtCoM().getLinearPart();
         if (EuclidCoreTools.isZero(force.getZ(), 1.0e-3))
         {
            jacobianToPack.set(posCoP, posCoM + 2, 0.0);
            jacobianToPack.set(posCoP + 1, posCoM + 2, 0.0);
         }
         else
         {
            jacobianToPack.set(posCoP, posCoM + 2, -2.0 * force.getX() / force.getZ());
            jacobianToPack.set(posCoP + 1, posCoM + 2, -2.0 * force.getY() / force.getZ());
         }
      }
   }

   private final Point3D currentCoMState = new Point3D();

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

      currentCoMState.set(stateIndexProvider.getCoMPosition(), stateVector);
      if (measuredLinearMomentum != null)
         linearMomentumResidual.set(stateIndexProvider.getLinearMomentum(), stateVector);
      angularMomentumResidual.set(stateIndexProvider.getAngularMomentum(), stateVector);

      if (stateIndexProvider.hasCoMPositionOffset())
      {
         currentCoMState.addX(stateVector.get(stateIndexProvider.getCoMPositionOffset(), 0));
         currentCoMState.addY(stateVector.get(stateIndexProvider.getCoMPositionOffset() + 1, 0));
      }

      if (measuredLinearMomentum != null && stateIndexProvider.hasLinearMomentumOffset())
         addEquals(linearMomentumResidual, stateVector, stateIndexProvider.getLinearMomentumOffset(), 0);

      centerOfMassPositionResidual.sub(measuredCoMPosition, currentCoMState);
      if (measuredLinearMomentum != null)
         linearMomentumResidual.sub(measuredLinearMomentum, linearMomentumResidual);
      angularMomentumResidual.sub(measuredAngularMomentum, angularMomentumResidual);

      centerOfMassPositionResidual.get(posCoM, residualToPack);
      if (measuredLinearMomentum != null)
         linearMomentumResidual.get(posLinMom, residualToPack);
      angularMomentumResidual.get(posAngMom, residualToPack);

      if (measuredCenterOfPressure != null)
      {
         computeCoP(currentCoMState, momentumRateCalculator.getTotalSpatialForceAtCoM(), centerOfPressureResidual);
         centerOfPressureResidual.sub(measuredCenterOfPressure, centerOfPressureResidual);
         centerOfPressureResidual.get(posCoP, residualToPack);
      }
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
      if (measuredCenterOfPressure != null)
         setDiagonalElements(posCoP, posCoP, 2, centerOfPressureVariance.getValue() * sqrtHz, noiseCovarianceToPack);
   }
}