package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.NativeFilterMatrixOps;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialForce;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * Following the paper titled: ""Humanoid Momentum Estimation Using Sensed Contact Wrenches".
 */
public class WrenchBasedMomentumStateUpdater implements MomentumStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final StateEstimator momentumEstimator;

   private final YoPoint3D measuredCoMPosition = new YoPoint3D("kinematicsBasedCoMPosition", registry);
   private final YoVector3D measuredLinearMomentum = new YoVector3D("kinematicsBasedLinearMomentum", registry);
   private final YoVector3D measuredAngularMomentum = new YoVector3D("kinematicsBasedAngularMomentum", registry);
   private final CenterOfMassReferenceFrame measuredCoMFrame;
   private final MomentumCalculator momentumCalculator;

   private final YoBoolean hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);

   private final RobotState robotState = new RobotState(null, Collections.emptyList());

   public WrenchBasedMomentumStateUpdater(JointReadOnly rootJoint, List<WrenchSensor> wrenchSensors, double dt, double gravity)
   {
      gravity = Math.abs(gravity);
      double mass = TotalMassCalculator.computeSubTreeMass(MultiBodySystemTools.getRootBody(rootJoint.getPredecessor()));
      measuredCoMFrame = new CenterOfMassReferenceFrame("comFrame", worldFrame, rootJoint.getPredecessor());

      MomentumKinematicsBasedSensor sensor = new MomentumKinematicsBasedSensor(measuredCoMPosition,
                                                                               measuredLinearMomentum,
                                                                               measuredAngularMomentum,
                                                                               dt,
                                                                               registry);
      MomentumState state = new MomentumState(measuredCoMFrame, wrenchSensors, mass, gravity, dt, registry);

      robotState.addState(state);
      momentumEstimator = new StateEstimator(Collections.singletonList(sensor), robotState, registry);

      momentumCalculator = new MomentumCalculator(rootJoint.getPredecessor());
   }

   @Override
   public void initialize()
   {
   }

   private final DMatrixRMaj stateVector = new DMatrixRMaj(9, 1);

   @Override
   public void update()
   {
      updateKinematicsBasedMomemtum();

      if (!hasBeenInitialized.getValue())
      {
         momentumEstimator.reset();
         stateVector.zero();
         measuredCoMPosition.get(0, stateVector);
         measuredLinearMomentum.get(3, stateVector);
         measuredAngularMomentum.get(6, stateVector);
         robotState.setStateVector(stateVector);
         hasBeenInitialized.set(true);
      }
      else
      {
         momentumEstimator.predict();
         momentumEstimator.correct();
      }
   }

   private final Momentum tempMomentum = new Momentum();

   private void updateKinematicsBasedMomemtum()
   {
      measuredCoMFrame.update();
      measuredCoMPosition.set(measuredCoMFrame.getTransformToRoot().getTranslation());
      tempMomentum.setReferenceFrame(measuredCoMFrame);
      momentumCalculator.computeAndPack(tempMomentum);
      measuredLinearMomentum.set(tempMomentum.getLinearPart());
      measuredAngularMomentum.set(tempMomentum.getAngularPart());
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   private static class MomentumKinematicsBasedSensor extends Sensor
   {
      private final int size = 6; // 3 for CoM position + 3 for angular momentum
      private final Point3DReadOnly measuredCoMPosition;
      private final Vector3DReadOnly measuredAngularMomentum;

      private final YoVector3D centerOfMassPositionResidual;
      private final YoVector3D angularMomentumResidual;

      private final double sqrtHz;
      private final DoubleProvider centerOfMassVariance;
      private final DoubleProvider angularMomentumVariance;

      public MomentumKinematicsBasedSensor(Point3DReadOnly measuredCoMPosition,
                                           Vector3DReadOnly measuredLinearMomentum,
                                           Vector3DReadOnly measuredAngularMomentum,
                                           double dt,
                                           YoRegistry registry)
      {
         this.measuredCoMPosition = measuredCoMPosition;
         this.measuredAngularMomentum = measuredAngularMomentum;

         sqrtHz = 1.0 / Math.sqrt(dt);
         centerOfMassVariance = FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, MathTools.square(0.0001));
         angularMomentumVariance = FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance", registry, MathTools.square(0.1));

         centerOfMassPositionResidual = new YoVector3D("centerOfMassPositionResidual", registry);
         angularMomentumResidual = new YoVector3D("angularMomentumResidual", registry);
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
       * / 1 0 0  0 0 0  0 0 0 \
       * | 0 1 0  0 0 0  0 0 0 |
       * | 0 0 1  0 0 0  0 0 0 |
       * | 0 0 0  0 0 0  1 0 0 |
       * | 0 0 0  0 0 0  0 1 0 |
       * \ 0 0 0  0 0 0  0 0 1 /
       * </pre>
       */
      @Override
      public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
      {
         jacobianToPack.reshape(size, robotState.getSize());
         jacobianToPack.zero();

         for (int i = 0; i < 3; i++)
         {
            jacobianToPack.set(i, i, 1.0);
            jacobianToPack.set(i + 3, i + 6, 1.0);
         }
      }

      private final DMatrixRMaj stateVector = new DMatrixRMaj(9, 1);

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

         centerOfMassPositionResidual.set(0, stateVector);
         angularMomentumResidual.set(6, stateVector);

         centerOfMassPositionResidual.sub(measuredCoMPosition, centerOfMassPositionResidual);
         angularMomentumResidual.sub(measuredAngularMomentum, angularMomentumResidual);

         centerOfMassPositionResidual.get(0, residualToPack);
         angularMomentumResidual.get(3, residualToPack);
      }

      @Override
      public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
      {
         noiseCovarianceToPack.reshape(getMeasurementSize(), getMeasurementSize());
         noiseCovarianceToPack.zero();

         for (int i = 0; i < 3; i++)
         {
            noiseCovarianceToPack.set(i, i, centerOfMassVariance.getValue() * sqrtHz);
            noiseCovarianceToPack.set(i + 3, i + 3, angularMomentumVariance.getValue() * sqrtHz);
         }
      }
   }

   private static class MomentumState extends State
   {
      private final int size = 9; // 3 for CoM Position + 6 for momentum
      private final DMatrixRMaj F = new DMatrixRMaj(size, size);
      private final DMatrixRMaj L;
      private final DMatrixRMaj FL = new DMatrixRMaj(size, size);
      private final DMatrixRMaj Qref;
      private final DMatrixRMaj Q = new DMatrixRMaj(size, size);
      private final List<WrenchSensor> wrenchSensors;
      private final double mass;
      private final double dt;
      private final double gravity;

      private final ReferenceFrame measuredCoMFrame;

      private final Wrench wrench = new Wrench();

      private final YoPoint3D centerOfMassPositionState;
      private final YoVector3D linearMomentumState;
      private final YoVector3D angularMomentumState;
      private final YoVector3D linearMomentumRateState;
      private final YoVector3D angularMomentumRateState;

      private final YoFixedFrameSpatialForce totalSpatialForceAtCoM;
      private final Vector3D comToFoot = new Vector3D();

      private final DoubleParameter forceSensorVariance;
      private final DoubleParameter torqueSensorVariance;

      private final double sqrtHz;

      public MomentumState(ReferenceFrame measuredCoMFrame, List<WrenchSensor> wrenchSensors, double mass, double gravity, double dt, YoRegistry registry)
      {
         this.measuredCoMFrame = measuredCoMFrame;
         this.wrenchSensors = wrenchSensors;
         this.mass = mass;
         this.gravity = gravity;
         this.dt = dt;

         sqrtHz = 1.0 / Math.sqrt(dt);
         Qref = CommonOps_DDRM.identity(6 * wrenchSensors.size());

         DMatrixRMaj identity3D = CommonOps_DDRM.identity(3);
         L = new DMatrixRMaj(size, 6 * wrenchSensors.size());

         for (int i = 0; i < wrenchSensors.size(); i++)
         {
            CommonOps_DDRM.insert(identity3D, L, 3, i * 6 + 0);
            CommonOps_DDRM.insert(identity3D, L, 6, i * 6 + 3);
         }

         centerOfMassPositionState = new YoPoint3D("centerOfMassPositionState", registry);
         linearMomentumState = new YoVector3D("linearMomentumState", registry);
         angularMomentumState = new YoVector3D("angularMomentumState", registry);
         linearMomentumRateState = new YoVector3D("linearMomentumRateState", registry);
         angularMomentumRateState = new YoVector3D("angularMomentumRateState", registry);

         totalSpatialForceAtCoM = new YoFixedFrameSpatialForce("totalSpatialForceAtCoM", measuredCoMFrame, registry);

         forceSensorVariance = new DoubleParameter("forceSensorVariance", registry, MathTools.square(0.06325));
         torqueSensorVariance = new DoubleParameter("torqueSensorVariance", registry, MathTools.square(0.0316));
      }

      @Override
      public String getName()
      {
         return "ExtendedMomentum";
      }

      @Override
      public void setStateVector(DMatrix1Row newState)
      {
         MatrixTools.checkMatrixDimensions(newState, getSize(), 1);
         centerOfMassPositionState.set(0, newState);
         linearMomentumState.set(3, newState);
         angularMomentumState.set(6, newState);
      }

      @Override
      public void getStateVector(DMatrix1Row stateVectorToPack)
      {
         stateVectorToPack.reshape(getSize(), 1);
         centerOfMassPositionState.get(0, stateVectorToPack);
         linearMomentumState.get(3, stateVectorToPack);
         angularMomentumState.get(6, stateVectorToPack);
      }

      @Override
      public int getSize()
      {
         return size;
      }

      /**
       * <pre>
       * x_CoM^predic = l/m * dt
       * l^predic     = lDot * dt
       * k^predic     = kDot * dt
       * lDot         = &sum; F^ext
       * kDot         = &sum; T^ext(CoM)
       * </pre>
       */
      @Override
      public void predict()
      {
         totalSpatialForceAtCoM.setToZero();

         for (int i = 0; i < wrenchSensors.size(); i++)
         {
            WrenchSensor wrenchSensor = wrenchSensors.get(i);
            wrenchSensor.getMeasuredWrench(wrench);
            wrench.changeFrame(measuredCoMFrame);
            totalSpatialForceAtCoM.add(wrench);

            comToFoot.sub(wrenchSensor.getMeasurementFrame().getTransformToRoot().getTranslation(), measuredCoMFrame.getTransformToRoot().getTranslation());
            insertSkewMatrix(6, i * 6, comToFoot, L);
         }

         linearMomentumRateState.set(totalSpatialForceAtCoM.getLinearPart());
         linearMomentumRateState.subZ(mass * gravity);
         angularMomentumRateState.set(totalSpatialForceAtCoM.getAngularPart());

         F.zero();
         for (int comIndex = 0; comIndex < 3; comIndex++)
            F.set(comIndex, 3 + comIndex, dt / mass);
         insertSkewMatrix(6, 0, dt, totalSpatialForceAtCoM.getLinearPart(), F);
         for (int i = 0; i < 9; i++)
            F.set(i, i, 1.0);
         CommonOps_DDRM.mult(F, L, FL);
         for (int sensorIndex = 0; sensorIndex < wrenchSensors.size(); sensorIndex++)
         {
            int startIndex = sensorIndex * 6;

            for (int i = 0; i < 3; i++)
            {
               int forceIndex = startIndex + i;
               int torqueIndex = forceIndex + 3;
               Qref.set(forceIndex, forceIndex, forceSensorVariance.getValue() * sqrtHz);
               Qref.set(torqueIndex, torqueIndex, torqueSensorVariance.getValue() * sqrtHz);
            }
         }
         NativeFilterMatrixOps.computeABAt(Q, FL, Qref);
         CommonOps_DDRM.scale(dt, Q);

         //         centerOfMassPositionState.scaleAdd(dt / mass, linearMomentumState, centerOfMassPositionState);
         //         centerOfMassPositionState.scaleAdd(0.5 * dt * dt / mass, linearMomentumRateState, centerOfMassPositionState);
         linearMomentumState.scaleAdd(dt, linearMomentumRateState, linearMomentumState);
         angularMomentumState.scaleAdd(dt, angularMomentumRateState, angularMomentumState);
         centerOfMassPositionState.scaleAdd(dt / mass, linearMomentumState, centerOfMassPositionState);
      }

      /**
       * <pre>
       *     /   0   0   0 m 0 0 0 0 0 \
       *     |   0   0   0 0 m 0 0 0 0 |
       *     |   0   0   0 0 0 m 0 0 0 |
       *     |   0   0   0 0 0 0 0 0 0 |
       * F = |   0   0   0 0 0 0 0 0 0 | * dt + I_9x9
       *     |   0   0   0 0 0 0 0 0 0 |
       *     |   0 -fz  fy 0 0 0 0 0 0 |
       *     |  fz   0 -fx 0 0 0 0 0 0 |
       *     \ -fy  fx   0 0 0 0 0 0 0 /
       * </pre>
       */
      @Override
      public void getFMatrix(DMatrix1Row fMatrixToPack)
      {
         fMatrixToPack.set(F);
      }

      /**
       * <pre>
       *       /        0             0             0      0 0 0 \
       *       |        0             0             0      0 0 0 |
       *       |        0             0             0      0 0 0 |
       * L_i = |        1             0             0      0 0 0 |
       *       |        0             1             0      0 0 0 |
       *       |        0             0             1      0 0 0 |
       *       |        0      -(pi_z - c_z)  (pi_y - c_y) 1 0 0 |
       *       |  (pi_z - c_z)        0      -(pi_x - c_x) 0 1 0 |
       *       \ -(pi_y - c_y)  (pi_x - c_x)        0      0 0 1 /
       * L = [L_0 L_1 ... L_N]
       * </pre>
       * 
       * <pre>
       * Q = F * L * Qc * L' * F' * dt
       * </pre>
       */
      @Override
      public void getQMatrix(DMatrix1Row noiseCovarianceToPack)
      {
         noiseCovarianceToPack.set(Q);
      }
   }

   /**
    * Inserts the following 3-by-3 skew-matrix in {@code matrix}:
    * 
    * <pre>
    *        /  0 -z  y \
    * skew = |  z  0 -x |
    *        \ -y  x  0 /
    * </pre>
    * 
    * where x, y, and z are the components of the given {@code vector}.
    */
   private static void insertSkewMatrix(int rowStart, int colStart, Tuple3DReadOnly vector, DMatrixRMaj matrix)
   {
      insertSkewMatrix(rowStart, colStart, vector.getX(), vector.getY(), vector.getZ(), matrix);
   }

   private static void insertSkewMatrix(int rowStart, int colStart, double scale, Tuple3DReadOnly vector, DMatrixRMaj matrix)
   {
      insertSkewMatrix(rowStart, colStart, scale * vector.getX(), scale * vector.getY(), scale * vector.getZ(), matrix);
   }

   /**
    * Inserts the following 3-by-3 skew-matrix in {@code matrix}:
    * 
    * <pre>
    *        /  0 -z  y \
    * skew = |  z  0 -x |
    *        \ -y  x  0 /
    * </pre>
    */
   private static void insertSkewMatrix(int rowStart, int colStart, double x, double y, double z, DMatrixRMaj matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(rowStart + 3, colStart + 3, matrix);

      matrix.set(rowStart + 0, colStart + 0, 0.0);
      matrix.set(rowStart + 0, colStart + 1, -z);
      matrix.set(rowStart + 0, colStart + 2, +y);
      matrix.set(rowStart + 1, colStart + 0, +z);
      matrix.set(rowStart + 1, colStart + 1, 0.0);
      matrix.set(rowStart + 1, colStart + 2, -x);
      matrix.set(rowStart + 2, colStart + 0, -y);
      matrix.set(rowStart + 2, colStart + 1, +x);
      matrix.set(rowStart + 2, colStart + 2, 0.0);
   }

   public static interface WrenchSensor
   {
      ReferenceFrame getMeasurementFrame();

      void getMeasuredWrench(Wrench wrenchToPack);
   }

   public static List<WrenchSensor> wrapFootSwitchInterfaces(List<FootSwitchInterface> footSwitchInterfaces)
   {
      return footSwitchInterfaces.stream().map(WrenchBasedMomentumStateUpdater::wrapFootSwitchInterface).collect(Collectors.toList());
   }

   public static WrenchSensor wrapFootSwitchInterface(FootSwitchInterface footSwitchInterface)
   {
      return new WrenchSensor()
      {
         @Override
         public ReferenceFrame getMeasurementFrame()
         {
            return footSwitchInterface.getMeasurementFrame();
         }

         @Override
         public void getMeasuredWrench(Wrench wrenchToPack)
         {
            footSwitchInterface.computeAndPackFootWrench(wrenchToPack);
         }
      };
   }
}
