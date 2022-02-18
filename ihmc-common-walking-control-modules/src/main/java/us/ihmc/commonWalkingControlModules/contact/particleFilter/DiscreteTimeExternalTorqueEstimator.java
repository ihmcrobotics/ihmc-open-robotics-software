package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

/**
 * Discrete-time formulation of the generalized momentum estimator. Based on the derivation in:
 * https://dspace.mit.edu/bitstream/handle/1721.1/120350/Main.pdf
 */
public class DiscreteTimeExternalTorqueEstimator implements ExternalTorqueEstimatorInterface
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoBoolean requestInitialize = new YoBoolean("requestInitialize", registry);
   private final double dt;

   private final YoDouble discreteTimeGain = new YoDouble("discreteTimeFilterGain", registry);
   private final YoDouble beta = new YoDouble("discreteTimeMomentumGain", registry);

   private final JointBasics[] joints;
   private final int dofs;
   private final DMatrixRMaj tau;
   private final DMatrixRMaj qd;
   private final DMatrixRMaj massMatrix;
   private final DMatrixRMaj massMatrixPrev;
   private final DMatrixRMaj massMatrixDot;
   private final DMatrixRMaj hqd;
   private final DMatrixRMaj hdqd;
   private final DMatrixRMaj coriolisGravityTerm;
   private final DMatrixRMaj estimatedExternalTorque;

   private final DMatrixRMaj sampledFilterValue;
   private final DMatrixRMaj runningFilteredValue;

   private final ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater;

   private final YoDouble[] yoObservedExternalJointTorque;
   private final YoDouble[] yoSimulatedTorqueSensingError;

   private boolean firstTick = true;

   public DiscreteTimeExternalTorqueEstimator(JointBasics[] joints,
                                              double dt,
                                              ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater,
                                              YoRegistry parentRegistry)
   {
      this.joints = joints;
      this.dt = dt;

      this.dynamicMatrixUpdater = dynamicMatrixUpdater;
      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
      this.tau = new DMatrixRMaj(dofs, 1);
      this.qd = new DMatrixRMaj(dofs, 1);
      this.coriolisGravityTerm = new DMatrixRMaj(dofs, 1);
      this.hqd = new DMatrixRMaj(dofs, 1);
      this.hdqd = new DMatrixRMaj(dofs, 1);
      this.massMatrix = new DMatrixRMaj(dofs, dofs);
      this.massMatrixPrev = new DMatrixRMaj(dofs, dofs);
      this.massMatrixDot = new DMatrixRMaj(dofs, dofs);
      this.sampledFilterValue = new DMatrixRMaj(dofs, 1);
      this.runningFilteredValue = new DMatrixRMaj(dofs, 1);
      this.estimatedExternalTorque = new DMatrixRMaj(dofs, 1);

      this.yoObservedExternalJointTorque = new YoDouble[dofs];
      this.yoSimulatedTorqueSensingError = new YoDouble[dofs];

      this.discreteTimeGain.set(computeDiscreteTimeGain(ExternalTorqueEstimator.defaultEstimatorGain, dt));
      this.beta.set(computeMomentumGain(discreteTimeGain.getValue(), dt));

      for (int i = 0; i < dofs; i++)
      {
         String nameSuffix = joints[i].getName();
         yoObservedExternalJointTorque[i] = new YoDouble("estimatedExternalTau_" + nameSuffix, registry);
         yoSimulatedTorqueSensingError[i] = new YoDouble("simulatedTauSensorError_" + nameSuffix, registry);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   /**
    * Computes discrete-time Z domain gain variable from continuous time cutoff frequency. See Sec. III-B in the above paper
    */
   private static double computeDiscreteTimeGain(double continuousTimeGain, double dt)
   {
      return Math.exp(-continuousTimeGain * dt);
   }

   /**
    * Momentum coefficient, see Sec. III-B in the above paper
    */
   private static double computeMomentumGain(double discreteTimeGain, double dt)
   {
      return (1 - discreteTimeGain) / (discreteTimeGain * dt);
   }

   @Override
   public void initialize()
   {
      firstTick = true;
      CommonOps_DDRM.fill(estimatedExternalTorque, 0.0);
      CommonOps_DDRM.fill(runningFilteredValue, 0.0);
   }

   @Override
   public void doControl()
   {
      if (requestInitialize.getValue())
      {
         initialize();
         requestInitialize.set(false);
      }

      massMatrixPrev.set(massMatrix);
      dynamicMatrixUpdater.update(massMatrix, coriolisGravityTerm, tau);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd);

      if (firstTick)
      {
         firstTick = false;
      }
      else
      {
         CommonOps_DDRM.subtract(massMatrix, massMatrixPrev, massMatrixDot);
         CommonOps_DDRM.scale(1.0 / dt, massMatrixDot);
      }

      for (int i = 0; i < dofs; i++)
      {
         tau.set(i, 0, tau.get(i, 0) - yoSimulatedTorqueSensingError[i].getValue());
      }

      CommonOps_DDRM.mult(massMatrix, qd, hqd);
      CommonOps_DDRM.mult(massMatrixDot, qd, hdqd);

      sampledFilterValue.set(hqd);
      CommonOps_DDRM.scale(beta.getValue(), sampledFilterValue);
      CommonOps_DDRM.addEquals(sampledFilterValue, tau);
      CommonOps_DDRM.subtractEquals(sampledFilterValue, coriolisGravityTerm);
      CommonOps_DDRM.addEquals(sampledFilterValue, hdqd);
      CommonOps_DDRM.scale(1.0 - discreteTimeGain.getValue(), sampledFilterValue);
      CommonOps_DDRM.scale(discreteTimeGain.getValue(), runningFilteredValue);
      CommonOps_DDRM.addEquals(runningFilteredValue, sampledFilterValue);

      estimatedExternalTorque.set(hqd);
      CommonOps_DDRM.scale(beta.getValue(), estimatedExternalTorque);
      CommonOps_DDRM.subtractEquals(estimatedExternalTorque, runningFilteredValue);

      for (int i = 0; i < dofs; i++)
      {
         yoObservedExternalJointTorque[i].set(estimatedExternalTorque.get(i, 0));
      }
   }

   @Override
   public DMatrixRMaj getEstimatedExternalTorque()
   {
      return estimatedExternalTorque;
   }

   @Override
   public void requestInitialize()
   {
      this.requestInitialize.set(true);
   }

   @Override
   public void setEstimatorGain(double estimatorGain)
   {
      this.discreteTimeGain.set(computeDiscreteTimeGain(estimatorGain, dt));
      this.beta.set(computeMomentumGain(discreteTimeGain.getValue(), dt));
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
