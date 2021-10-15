package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
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
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.MomentumStateUpdater;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector2D;
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

   private final DMatrixRMaj stateVector = new DMatrixRMaj(MomentumState.size, 1);

   @Override
   public void update()
   {
      updateKinematicsBasedMomemtum();

      if (!hasBeenInitialized.getValue())
      {
         momentumEstimator.reset();
         stateVector.zero();
         measuredCoMPosition.get(MomentumState.posCoM, stateVector);
         measuredLinearMomentum.get(MomentumState.posLinMom, stateVector);
         measuredAngularMomentum.get(MomentumState.posAngMom, stateVector);
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
      private static final int posCoM = 0;
      private static final int posLinMom = 3;
      private static final int posAngMom = 6;
      private static final int size = 9; // 3 for CoM position + 6 for momentum
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

      public MomentumKinematicsBasedSensor(Point3DReadOnly measuredCoMPosition,
                                           Vector3DReadOnly measuredLinearMomentum,
                                           Vector3DReadOnly measuredAngularMomentum,
                                           double dt,
                                           YoRegistry registry)
      {
         this.measuredCoMPosition = measuredCoMPosition;
         this.measuredLinearMomentum = measuredLinearMomentum;
         this.measuredAngularMomentum = measuredAngularMomentum;

         sqrtHz = 1.0 / Math.sqrt(dt);
         centerOfMassVariance = FilterTools.findOrCreate("kinematicsBasedCenterOfMassVariance", registry, MathTools.square(0.001));
         linearMomentumVariance = FilterTools.findOrCreate("kinematicsBasedLinearMomentumVariance", registry, MathTools.square(1.0));
         angularMomentumVariance = FilterTools.findOrCreate("kinematicsBasedAngularMomentumVariance", registry, MathTools.square(1.0));

         centerOfMassPositionResidual = new YoVector3D("centerOfMassPositionResidual", registry);
         linearMomentumResidual = new YoVector3D("linearMomentumResidual", registry);
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

         for (int i = 0; i < 3; i++)
         {
            jacobianToPack.set(i + posCoM, i + MomentumState.posCoM, 1.0);
            jacobianToPack.set(i + posLinMom, i + MomentumState.posLinMom, 1.0);
            jacobianToPack.set(i + posLinMom, i + MomentumState.posLinMomOff, 1.0);
            jacobianToPack.set(i + posAngMom, i + MomentumState.posAngMom, 1.0);
         }

         for (int i = 0; i < 2; i++)
         {
            jacobianToPack.set(i + posCoM, i + MomentumState.posCoMOff, 1.0);
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

         centerOfMassPositionResidual.set(MomentumState.posCoM, stateVector);
         linearMomentumResidual.set(MomentumState.posLinMom, stateVector);
         angularMomentumResidual.set(MomentumState.posAngMom, stateVector);

         for (int i = 0; i < 2; i++)
            centerOfMassPositionResidual.setElement(i, centerOfMassPositionResidual.getElement(i) + stateVector.get(MomentumState.posCoMOff + i, 0));
         for (int i = 0; i < 3; i++)
            linearMomentumResidual.setElement(i, linearMomentumResidual.getElement(i) + stateVector.get(MomentumState.posLinMomOff + i, 0));

         centerOfMassPositionResidual.sub(measuredCoMPosition, centerOfMassPositionResidual);
         linearMomentumResidual.sub(measuredLinearMomentum, linearMomentumResidual);
         angularMomentumResidual.sub(measuredAngularMomentum, angularMomentumResidual);

         centerOfMassPositionResidual.get(posCoM, residualToPack);
         linearMomentumResidual.get(posLinMom, residualToPack);
         angularMomentumResidual.get(posAngMom, residualToPack);
      }

      @Override
      public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
      {
         noiseCovarianceToPack.reshape(getMeasurementSize(), getMeasurementSize());
         noiseCovarianceToPack.zero();

         for (int i = 0; i < 3; i++)
         {
            noiseCovarianceToPack.set(i + posCoM, i + posCoM, centerOfMassVariance.getValue() * sqrtHz);
            noiseCovarianceToPack.set(i + posLinMom, i + posLinMom, linearMomentumVariance.getValue() * sqrtHz);
            noiseCovarianceToPack.set(i + posAngMom, i + posAngMom, angularMomentumVariance.getValue() * sqrtHz);
         }
      }
   }

   private static class MomentumState extends State
   {
      private static final int posCoM = 0;
      private static final int posLinMom = 3;
      private static final int posAngMom = 6;
      private static final int posCoMOff = 9;
      private static final int posLinMomOff = 11;
      private static final int size = 14; // 3 for CoM Position + 6 for momentum + 2 CoM offset + 3 Lin. Momentum offset
      private final DMatrixRMaj F = new DMatrixRMaj(size, size);
      private final DMatrixRMaj L;
      private final DMatrixRMaj FL = new DMatrixRMaj(size, size);
      private final DMatrixRMaj Qref;
      private final DMatrixRMaj Q = new DMatrixRMaj(size, size);
      private final List<WrenchSensor> wrenchSensors;
      private final double mass;
      private final double dt;
      private final double gravity;

      private final ReferenceFrame correctedCoMFrame;

      private final Wrench wrench = new Wrench();

      private final YoPoint3D centerOfMassPositionState;
      private final YoVector3D linearMomentumState;
      private final YoVector3D angularMomentumState;
      private final YoVector3D linearMomentumRateState;
      private final YoVector3D angularMomentumRateState;

      private final YoVector2D centerOfMassOffsetState;
      private final YoVector3D linearMomentumOffsetState;

      private final YoFixedFrameSpatialForce totalSpatialForceAtCoM;
      private final Vector3D comToFoot = new Vector3D();

      private final DoubleParameter forceSensorVariance;
      private final DoubleParameter torqueSensorVariance;
      private final DoubleParameter centerOfMassOffsetVariance;
      private final DoubleParameter linearMomentumOffsetVariance;

      private final double sqrtHz;
      private final int nWrenchSensors;

      public MomentumState(ReferenceFrame measuredCoMFrame, List<WrenchSensor> wrenchSensors, double mass, double gravity, double dt, YoRegistry registry)
      {
         this.wrenchSensors = wrenchSensors;
         this.mass = mass;
         this.gravity = gravity;
         this.dt = dt;
         nWrenchSensors = wrenchSensors.size();
         sqrtHz = 1.0 / Math.sqrt(dt);
         Qref = CommonOps_DDRM.identity(6 * nWrenchSensors + 5);

         DMatrixRMaj identity3D = CommonOps_DDRM.identity(3);
         L = new DMatrixRMaj(size, 6 * nWrenchSensors + 5);

         for (int i = 0; i < wrenchSensors.size(); i++)
         {
            CommonOps_DDRM.insert(identity3D, L, posLinMom, i * 6 + 0);
            CommonOps_DDRM.insert(identity3D, L, posAngMom, i * 6 + 3);
         }
         for (int i = 0; i < 2; i++)
            L.set(posCoMOff + i, 6 * nWrenchSensors + i, 1.0);
         for (int i = 0; i < 3; i++)
            L.set(posLinMomOff + i, 6 * nWrenchSensors + i + 2, 1.0);

         CommonOps_DDRM.setIdentity(F);

         for (int i = 0; i < 3; i++)
            F.set(posCoM + i, posLinMom + i, dt / mass);

         centerOfMassPositionState = new YoPoint3D("centerOfMassPositionState", registry);
         linearMomentumState = new YoVector3D("linearMomentumState", registry);
         angularMomentumState = new YoVector3D("angularMomentumState", registry);
         linearMomentumRateState = new YoVector3D("linearMomentumRateState", registry);
         angularMomentumRateState = new YoVector3D("angularMomentumRateState", registry);
         centerOfMassOffsetState = new YoVector2D("centerOfMassOffsetState", registry);
         linearMomentumOffsetState = new YoVector3D("linearMomentumOffsetState", registry);

         correctedCoMFrame = new ReferenceFrame("correctedCoMFrame", measuredCoMFrame)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               Vector3DBasics translation = transformToParent.getTranslation();
               translation.set(centerOfMassOffsetState);
               translation.negate();
            }
         };

         totalSpatialForceAtCoM = new YoFixedFrameSpatialForce("totalSpatialForceAtCoM", correctedCoMFrame, registry);

         forceSensorVariance = new DoubleParameter("forceSensorVariance", registry, MathTools.square(0.06325));
         torqueSensorVariance = new DoubleParameter("torqueSensorVariance", registry, MathTools.square(0.0316));
         centerOfMassOffsetVariance = new DoubleParameter("centerOfMassOffsetVariance", registry, MathTools.square(1.0));
         linearMomentumOffsetVariance = new DoubleParameter("linearMomentumOffsetVariance", registry, MathTools.square(1.0));
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
         centerOfMassPositionState.set(posCoM, newState);
         linearMomentumState.set(posLinMom, newState);
         angularMomentumState.set(posAngMom, newState);
         centerOfMassOffsetState.set(posCoMOff, newState);
         linearMomentumOffsetState.set(posLinMomOff, newState);
      }

      @Override
      public void getStateVector(DMatrix1Row stateVectorToPack)
      {
         stateVectorToPack.reshape(getSize(), 1);
         centerOfMassPositionState.get(posCoM, stateVectorToPack);
         linearMomentumState.get(posLinMom, stateVectorToPack);
         angularMomentumState.get(posAngMom, stateVectorToPack);
         centerOfMassOffsetState.get(posCoMOff, stateVectorToPack);
         linearMomentumOffsetState.get(posLinMomOff, stateVectorToPack);
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
         correctedCoMFrame.update();
         totalSpatialForceAtCoM.setToZero();

         for (int i = 0; i < nWrenchSensors; i++)
         {
            WrenchSensor wrenchSensor = wrenchSensors.get(i);
            wrenchSensor.getMeasuredWrench(wrench);
            wrench.changeFrame(correctedCoMFrame);
            totalSpatialForceAtCoM.add(wrench);
         }

         linearMomentumRateState.set(totalSpatialForceAtCoM.getLinearPart());
         linearMomentumRateState.subZ(mass * gravity);
         angularMomentumRateState.set(totalSpatialForceAtCoM.getAngularPart());

         //         centerOfMassPositionState.scaleAdd(dt / mass, linearMomentumState, centerOfMassPositionState);
         //         centerOfMassPositionState.scaleAdd(0.5 * dt * dt / mass, linearMomentumRateState, centerOfMassPositionState);
         linearMomentumState.scaleAdd(dt, linearMomentumRateState, linearMomentumState);
         angularMomentumState.scaleAdd(dt, angularMomentumRateState, angularMomentumState);
         centerOfMassPositionState.scaleAdd(dt / mass, linearMomentumState, centerOfMassPositionState);

         updateF();
         updateQ();
      }

      private void updateF()
      {
         for (int sensorIndex = 0; sensorIndex < nWrenchSensors; sensorIndex++)
         {
            Vector3DBasics sensorPosition = wrenchSensors.get(sensorIndex).getMeasurementFrame().getTransformToRoot().getTranslation();
            Vector3DBasics comPosition = correctedCoMFrame.getTransformToRoot().getTranslation();
            comToFoot.sub(sensorPosition, comPosition);
            insertSkewMatrix(posAngMom, sensorIndex * 6, comToFoot, L);
         }

         insertSkewMatrix(posAngMom, posCoM, dt, totalSpatialForceAtCoM.getLinearPart(), F);
      }

      private void updateQ()
      {
         CommonOps_DDRM.mult(F, L, FL);

         for (int sensorIndex = 0; sensorIndex < nWrenchSensors; sensorIndex++)
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

         int index = 6 * nWrenchSensors;

         for (int i = 0; i < 2; i++)
         {
            Qref.set(index, index, centerOfMassOffsetVariance.getValue() * sqrtHz);
            index++;
         }
         for (int i = 0; i < 3; i++)
         {
            Qref.set(index, index, linearMomentumOffsetVariance.getValue() * sqrtHz);
            index++;
         }
         NativeFilterMatrixOps.computeABAt(Q, FL, Qref);
         CommonOps_DDRM.scale(dt, Q);
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
