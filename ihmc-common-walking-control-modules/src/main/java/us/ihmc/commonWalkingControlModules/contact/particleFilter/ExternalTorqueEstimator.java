package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

/**
 * Module to estimate unknown external contact. This class estimates an array of joint torques based on the discrepancy between
 * the expected and actual system behavior. This module estimates the tau_ext variable in the below paper, i.e. the "generalized external forces", which
 * can in turn be used to estimate taskspace forces in a number of different ways.
 *
 * Implementation based on Haddadin, et. al:
 * <a href="www.repo.uni-hannover.de/bitstream/handle/123456789/3543/VorndammeSchHad2017_accepted.pdf">Collision Detection, Isolation and Identification for Humanoids</a>
 */
public class ExternalTorqueEstimator implements ExternalTorqueEstimatorInterface
{
   static final double defaultEstimatorGain = 0.7;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoBoolean requestInitialize = new YoBoolean("requestInitialize", registry);

   private final YoDouble estimatedExternalTorqueMagnitude = new YoDouble("estimatedExternalTorqueMagnitude", registry);
   private final YoDouble estimationGain = new YoDouble("estimationGain", registry);
   private final double dt;

   private final JointBasics[] joints;
   private final int dofs;
   private final DMatrixRMaj tau;
   private final DMatrixRMaj qd;

   private final DMatrixRMaj currentIntegrandValue;
   private final DMatrixRMaj currentIntegratedValue;
   private final DMatrixRMaj estimatedExternalTorque;
   private final DMatrixRMaj hqd0;
   private final DMatrixRMaj hqd;
   private final DMatrixRMaj massMatrix;
   private final DMatrixRMaj coriolisMatrix;
   private final DMatrixRMaj gravityMatrix;

   private final ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater;

   private final YoDouble[] yoObservedExternalJointTorque;
   private final YoDouble[] yoSimulatedTorqueSensingError;

   public ExternalTorqueEstimator(JointBasics[] joints,
                                  double dt,
                                  ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater,
                                  YoRegistry parentRegistry)
   {
      this.joints = joints;
      this.dt = dt;
      this.estimationGain.set(defaultEstimatorGain);

      this.dynamicMatrixUpdater = dynamicMatrixUpdater;
      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.currentIntegrandValue = new DMatrixRMaj(dofs, 1);
      this.currentIntegratedValue = new DMatrixRMaj(dofs, 1);
      this.estimatedExternalTorque = new DMatrixRMaj(dofs, 1);
      this.hqd = new DMatrixRMaj(dofs, 1);
      this.massMatrix = new DMatrixRMaj(dofs, dofs);
      this.coriolisMatrix = new DMatrixRMaj(dofs, dofs);
      this.gravityMatrix = new DMatrixRMaj(dofs, 1);
      this.tau = new DMatrixRMaj(dofs, 1);
      this.qd = new DMatrixRMaj(dofs, 1);
      this.hqd0 = new DMatrixRMaj(dofs, 1);

      this.yoObservedExternalJointTorque = new YoDouble[dofs];
      this.yoSimulatedTorqueSensingError = new YoDouble[dofs];

      boolean hasFloatingBase = joints[0] instanceof FloatingJointBasics;
      for (int i = 0; i < dofs; i++)
      {
         String nameSuffix;
         if (i < 6 && hasFloatingBase)
         {
            nameSuffix = (i < 3 ? "Ang" : "Lin") + Axis3D.values[i % 3];
         }
         else
         {
            nameSuffix = joints[i + (hasFloatingBase ? -5 : 0)].getName();
         }

         yoObservedExternalJointTorque[i] = new YoDouble("estimatedExternalTau_" + nameSuffix, registry);
         yoSimulatedTorqueSensingError[i] = new YoDouble("simulatedTauSensorError_" + nameSuffix, registry);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      CommonOps_DDRM.fill(estimatedExternalTorque, 0.0);
      CommonOps_DDRM.fill(currentIntegratedValue, 0.0);
   }

   @Override
   public void doControl()
   {
      if (requestInitialize.getValue())
      {
         initialize();
         requestInitialize.set(false);
      }

      try
      {
         computeForceEstimate();
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }
   }

   private void computeForceEstimate()
   {
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd);
      dynamicMatrixUpdater.update(massMatrix, coriolisMatrix, gravityMatrix, tau);

      for (int i = 0; i < dofs; i++)
      {
         tau.set(i, 0, tau.get(i, 0) - yoSimulatedTorqueSensingError[i].getValue());
      }

      // update integral
      currentIntegrandValue.set(tau);
      CommonOps_DDRM.subtractEquals(currentIntegrandValue, gravityMatrix);
      CommonOps_DDRM.multAddTransA(coriolisMatrix, qd, currentIntegrandValue);
      CommonOps_DDRM.addEquals(currentIntegrandValue, estimatedExternalTorque);
      CommonOps_DDRM.addEquals(currentIntegratedValue, dt, currentIntegrandValue);

      // calculate observed external joint torque
      CommonOps_DDRM.mult(massMatrix, qd, hqd);
      CommonOps_DDRM.subtract(hqd, hqd0, estimatedExternalTorque);
      CommonOps_DDRM.subtractEquals(estimatedExternalTorque, currentIntegratedValue);
      CommonOps_DDRM.scale(estimationGain.getDoubleValue(), estimatedExternalTorque);

      double estimatedExternalTorqueMagnitude = 0.0;
      for (int i = 0; i < dofs; i++)
      {
         yoObservedExternalJointTorque[i].set(estimatedExternalTorque.get(i, 0));
         estimatedExternalTorqueMagnitude += MathTools.square(yoObservedExternalJointTorque[i].getDoubleValue());
      }

      this.estimatedExternalTorqueMagnitude.set(Math.sqrt(estimatedExternalTorqueMagnitude));
   }

   @Override
   public DMatrixRMaj getEstimatedExternalTorque()
   {
      return estimatedExternalTorque;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void requestInitialize()
   {
      this.requestInitialize.set(true);
   }

   @Override
   public void setEstimatorGain(double estimatorGain)
   {
      this.estimationGain.set(estimatorGain);
   }

   @Override
   public double getEstimatedExternalTorqueMagnitude()
   {
      return estimatedExternalTorqueMagnitude.getDoubleValue();
   }
}
