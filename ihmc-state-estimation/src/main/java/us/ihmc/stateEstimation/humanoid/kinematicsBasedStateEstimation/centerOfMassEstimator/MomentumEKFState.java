package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.insertSkewMatrix;
import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.setDiagonalElements;

import java.util.List;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.NativeFilterMatrixOps;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.WrenchSensor;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MomentumEKFState extends State
{
   private final DMatrixRMaj F;
   private final DMatrixRMaj L;
   private final DMatrixRMaj FL;
   private final DMatrixRMaj Qref;
   private final DMatrixRMaj Q;
   private final List<WrenchSensor> wrenchSensors;
   private final double mass;
   private final double dt;

   private final ReferenceFrame correctedCoMFrame;

   private final WrenchBasedMomentumRateCalculator momentumRateCalculator;

   private final YoPoint3D centerOfMassPositionState;
   private final YoVector3D linearMomentumState;
   private final YoVector3D angularMomentumState;

   private final YoVector2D centerOfMassOffsetState;
   private final YoVector3D linearMomentumOffsetState;

   private final Vector3D comToFoot = new Vector3D();

   private final DoubleParameter forceSensorVariance;
   private final DoubleParameter torqueSensorVariance;
   private final DoubleParameter centerOfMassOffsetVariance;
   private final DoubleParameter linearMomentumOffsetVariance;

   private final double sqrtHz;
   private final int nWrenchSensors;

   private final int posCoM;
   private final int posLinMom;
   private final int posAngMom;
   private final int posCoMOff;
   private final int posLinMomOff;
   private final int size;
   private final boolean hasCoMOffset;
   private final boolean hasLinMomOffset;

   public MomentumEKFState(MomentumEKFEstimatorIndexProvider indexProvider,
                           ReferenceFrame measuredCoMFrame,
                           List<WrenchSensor> wrenchSensors,
                           double mass,
                           double gravity,
                           double dt,
                           YoRegistry registry)
   {
      this.wrenchSensors = wrenchSensors;
      this.mass = mass;
      this.dt = dt;

      posCoM = indexProvider.getCoMPosition();
      posLinMom = indexProvider.getLinearMomentum();
      posAngMom = indexProvider.getAngularMomentum();
      posCoMOff = indexProvider.getCoMPositionOffset();
      posLinMomOff = indexProvider.getLinearMomentumOffset();
      size = indexProvider.getSize();
      hasCoMOffset = indexProvider.hasCoMPositionOffset();
      hasLinMomOffset = indexProvider.hasLinearMomentumOffset();

      nWrenchSensors = wrenchSensors.size();
      sqrtHz = 1.0 / Math.sqrt(dt);
      Qref = CommonOps_DDRM.identity(6 * nWrenchSensors + 5);

      F = new DMatrixRMaj(size, size);
      FL = new DMatrixRMaj(size, size);
      Q = new DMatrixRMaj(size, size);

      DMatrixRMaj identity3D = CommonOps_DDRM.identity(3);
      L = new DMatrixRMaj(size, 6 * nWrenchSensors + 5);

      for (int i = 0; i < wrenchSensors.size(); i++)
      {
         CommonOps_DDRM.insert(identity3D, L, posLinMom, i * 6 + 0);
         CommonOps_DDRM.insert(identity3D, L, posAngMom, i * 6 + 3);
      }
      if (hasCoMOffset)
      {
         for (int i = 0; i < 2; i++)
            L.set(posCoMOff + i, 6 * nWrenchSensors + i, 1.0);
      }
      if (hasLinMomOffset)
      {
         for (int i = 0; i < 3; i++)
            L.set(posLinMomOff + i, 6 * nWrenchSensors + i + 2, 1.0);
      }

      CommonOps_DDRM.setIdentity(F);

      for (int i = 0; i < 3; i++)
         F.set(posCoM + i, posLinMom + i, dt / mass);

      centerOfMassPositionState = new YoPoint3D("centerOfMassPositionState", registry);
      linearMomentumState = new YoVector3D("linearMomentumState", registry);
      angularMomentumState = new YoVector3D("angularMomentumState", registry);
      centerOfMassOffsetState = hasCoMOffset ? new YoVector2D("centerOfMassOffsetState", registry) : null;
      linearMomentumOffsetState = hasLinMomOffset ? new YoVector3D("linearMomentumOffsetState", registry) : null;

      correctedCoMFrame = new ReferenceFrame("correctedCoMFrame", measuredCoMFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            if (hasCoMOffset)
            {
               Vector3DBasics translation = transformToParent.getTranslation();
               translation.set(centerOfMassOffsetState);
               translation.negate();
            }
         }
      };

      momentumRateCalculator = new WrenchBasedMomentumRateCalculator(correctedCoMFrame, wrenchSensors, mass, gravity, registry);

      forceSensorVariance = new DoubleParameter("forceSensorVariance", registry, MathTools.square(0.06325));
      torqueSensorVariance = new DoubleParameter("torqueSensorVariance", registry, MathTools.square(0.0316));
      centerOfMassOffsetVariance = hasCoMOffset ? new DoubleParameter("centerOfMassOffsetVariance", registry, MathTools.square(1.0)) : null;
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
      if (hasCoMOffset)
         centerOfMassOffsetState.set(posCoMOff, newState);
      if (hasLinMomOffset)
         linearMomentumOffsetState.set(posLinMomOff, newState);
   }

   @Override
   public void getStateVector(DMatrix1Row stateVectorToPack)
   {
      stateVectorToPack.reshape(getSize(), 1);
      centerOfMassPositionState.get(posCoM, stateVectorToPack);
      linearMomentumState.get(posLinMom, stateVectorToPack);
      angularMomentumState.get(posAngMom, stateVectorToPack);
      if (hasCoMOffset)
         centerOfMassOffsetState.get(posCoMOff, stateVectorToPack);
      if (hasLinMomOffset)
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
      momentumRateCalculator.update();

      //         centerOfMassPositionState.scaleAdd(dt / mass, linearMomentumState, centerOfMassPositionState);
      //         centerOfMassPositionState.scaleAdd(0.5 * dt * dt / mass, linearMomentumRateState, centerOfMassPositionState);
      linearMomentumState.scaleAdd(dt, momentumRateCalculator.getLinearMomentumRateState(), linearMomentumState);
      angularMomentumState.scaleAdd(dt, momentumRateCalculator.getAngularMomentumRateState(), angularMomentumState);
      centerOfMassPositionState.scaleAdd(dt / mass, linearMomentumState, centerOfMassPositionState);

      updateF();
      updateQ();
   }

   private void updateF()
   {
      insertSkewMatrix(posAngMom, posCoM, dt, momentumRateCalculator.getTotalSpatialForceAtCoM().getLinearPart(), F);
   }

   private void updateQ()
   {
      for (int sensorIndex = 0; sensorIndex < nWrenchSensors; sensorIndex++)
      {
         Vector3DBasics sensorPosition = wrenchSensors.get(sensorIndex).getMeasurementFrame().getTransformToRoot().getTranslation();
         Vector3DBasics comPosition = correctedCoMFrame.getTransformToRoot().getTranslation();
         comToFoot.sub(sensorPosition, comPosition);
         insertSkewMatrix(posAngMom, sensorIndex * 6, comToFoot, L);
      }

      CommonOps_DDRM.mult(F, L, FL);

      int index = 0;
      for (int sensorIndex = 0; sensorIndex < nWrenchSensors; sensorIndex++)
      {
         setDiagonalElements(index, index, 3, forceSensorVariance.getValue() * sqrtHz, Qref);
         index += 3;
         setDiagonalElements(index, index, 3, torqueSensorVariance.getValue() * sqrtHz, Qref);
         index += 3;
      }

      if (hasCoMOffset)
      {
         setDiagonalElements(index, index, 2, centerOfMassOffsetVariance.getValue() + sqrtHz, Qref);
         index += 2;
      }

      if (hasLinMomOffset)
      {
         setDiagonalElements(index, index, 3, linearMomentumOffsetVariance.getValue() + sqrtHz, Qref);
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

   public WrenchBasedMomentumRateCalculator getMomentumRateCalculator()
   {
      return momentumRateCalculator;
   }

   public ReferenceFrame getCorrectedCoMFrame()
   {
      return correctedCoMFrame;
   }

   public YoPoint3D getCenterOfMassPositionState()
   {
      return centerOfMassPositionState;
   }

   public YoVector3D getLinearMomentumState()
   {
      return linearMomentumState;
   }

   public YoVector3D getAngularMomentumState()
   {
      return angularMomentumState;
   }

   public YoVector2D getCenterOfMassOffsetState()
   {
      return centerOfMassOffsetState;
   }

   public YoVector3D getLinearMomentumOffsetState()
   {
      return linearMomentumOffsetState;
   }
}