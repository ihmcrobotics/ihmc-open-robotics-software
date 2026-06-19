package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoMatrix;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * An implementation of the {@link ExtendedKalmanFilter} for estimating inertial parameters of a robot in a physically-consistent manner (no negative masses or
 * invalid inertia matrices).
 * <p>
 * Features of the filter:
 * <li> The process model of the filter is how the inertial parameters are expected to change over time. As we cannot predict this, we consider the inertial
 * parameters to be constant and therefore process model is the identity mapping.
 * <li> The measurement of the filter is the torque contribution from the inertial parameters being estimated. We form a model of this by totaling up the other
 * torque contributions: the contribution from the current estimate of the inertial parameters, the contribution from the nominal inertial parameters that are
 * considered known and fixed, the contribution from the contact wrenches, and the contribution from a bias that approximates model mismatch.
 * <li> The measurement model uses a physically-consistent parameterization of rigid bodies that ensures that: mass is always positive, moment of inertia is
 * always positive definite, and the principal moments of inertia satisfy a triangle inequality.
 * </p>
 *
 * @author James Foster
 */
class InertialPhysicallyConsistentKalmanFilter extends ExtendedKalmanFilter implements OnlineInertialEstimator
{
   private final DMatrixRMaj IDENTITY;

   // Optional Tikhonov "soft constraint" pseudo-measurement (z=0, H=I, covariance R_prior) applied after the
   // torque update each tick: pulls weakly-observable parameter directions toward theta=0 (nominal) while
   // strongly-observed ones (mass) are nearly untouched. Available only in generalized mode (theta=0 = nominal).
   // The enable flag and the four per-parameter prior variances are exposed as YoVariables for LIVE tuning;
   // R_prior is rebuilt from them each tick. Initialized from getParameterPriorVariance() (null -> start disabled).
   private final boolean priorAvailable;
   private final YoBoolean tikhonovPriorEnabled;
   private final YoDouble tikhonovPriorVarianceAlpha; // mass (loose -> data dominates)
   private final YoDouble tikhonovPriorVarianceD;     // inertia diagonal
   private final YoDouble tikhonovPriorVarianceS;     // inertia off-diagonal
   private final YoDouble tikhonovPriorVarianceT;     // first moment / CoM (tight -> held near nominal)
   private final DMatrixRMaj priorCovarianceDiag;   // R_prior (full state size, block-diagonal per body)
   private final DMatrixRMaj priorInnovationCovariance; // S = P + R_prior
   private final DMatrixRMaj priorInnovationCovarianceInv;
   private final DMatrixRMaj priorGain;             // K = P S^-1
   private final DMatrixRMaj priorThetaFull;        // gathered theta of all bodies (the EKF covariance is theta-space)
   private final DMatrixRMaj priorStateContainer;
   private final DMatrixRMaj priorCovarianceContainer;

   private final DMatrixRMaj torqueFromNominal;
   private final DMatrixRMaj torqueFromBias;

   private final DMatrixRMaj regressor;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   private int nBodies;
   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final DMatrixRMaj measurementJacobianBlock;
   private final DMatrixRMaj measurementJacobianContainer;

   private final DMatrixRMaj regressorBlock;

   private final DMatrixRMaj parameterThetaBasisContainer;

   private final DMatrixRMaj kalmanGainBlockContainer;

   private final DMatrixRMaj measurement;

   private final AlphaFilteredYoMatrix filteredResidual;

   public InertialPhysicallyConsistentKalmanFilter(FullRobotModel model, InertialEstimationParameters parameters, DoubleProvider dt, YoRegistry parentRegistry)
   {
      super(parameters.getURDFParameters(parameters.getBasisSets()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray())));

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      Set<SpatialInertiaBasisOption>[] basisSets = parameters.getBasisSets();
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      IDENTITY = CommonOps_DDRM.identity(partitionSizes[0]);

      torqueFromNominal = new DMatrixRMaj(nDoFs, 1);
      torqueFromBias = new DMatrixRMaj(nDoFs, 1);

      regressor = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(Wrench.SIZE, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(Wrench.SIZE, 1));
      }

      nBodies = 0;
      RigidBodyReadOnly[] modelBodies = model.getRootBody().subtreeArray();
      for (int i = 0; i < modelBodies.length; ++i)
      {
         if (basisSets[i].isEmpty())
            continue;

         if (basisSets[i].size() != RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY)
         {
            throw new RuntimeException("This filter requires the full body to be estimated, not just certain parameters in the body.");
         }
         else
         {
            RigidBodyInertialParameters bodyParameters = new RigidBodyInertialParameters(modelBodies[i].getInertia());
            // Generalized (Eq. (11)) parameterization: estimate a physically-consistent deviation from the
            // known nominal inertia of this body (theta = 0 is the nominal). Off by default (absolute Eq. (1)).
            if (parameters.useGeneralizedPhysicalConsistency())
               bodyParameters.enableGeneralizedParameterization();
            inertialParameters.add(bodyParameters);
            inertialParametersPiBasisWatchers.add(new YoMatrix("pi_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForPiBasis(), null, registry));
            inertialParametersThetaBasisWatchers.add(new YoMatrix("theta_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForThetaBasis(), null, registry));
            nBodies++;
         }
      }

      // Tikhonov soft-constraint prior toward nominal (theta=0). Only meaningful with the generalized
      // parameterization. The enable flag + four per-parameter variances are YoVariables so they can be tuned
      // LIVE; R_prior is rebuilt from them each tick. getParameterPriorVariance() (Theta ordering
      // [alpha,d,d,d,s,s,s,t,t,t]) seeds the initial values and whether it starts enabled (null -> disabled).
      int stateSize = partitionSizes[0];
      priorAvailable = parameters.useGeneralizedPhysicalConsistency();
      double[] perBodyPriorVariance = parameters.getParameterPriorVariance();
      if (perBodyPriorVariance != null && perBodyPriorVariance.length != RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY)
         throw new RuntimeException("getParameterPriorVariance() must have length " + RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      if (priorAvailable)
      {
         tikhonovPriorEnabled = new YoBoolean("tikhonovPriorEnabled", registry);
         tikhonovPriorEnabled.set(perBodyPriorVariance != null);
         tikhonovPriorVarianceAlpha = new YoDouble("tikhonovPriorVarianceAlpha", registry);
         tikhonovPriorVarianceD = new YoDouble("tikhonovPriorVarianceD", registry);
         tikhonovPriorVarianceS = new YoDouble("tikhonovPriorVarianceS", registry);
         tikhonovPriorVarianceT = new YoDouble("tikhonovPriorVarianceT", registry);
         // index map within a body: 0=alpha, 1..3=d, 4..6=s, 7..9=t
         tikhonovPriorVarianceAlpha.set(perBodyPriorVariance != null ? perBodyPriorVariance[0] : 1.0e6);
         tikhonovPriorVarianceD.set(perBodyPriorVariance != null ? perBodyPriorVariance[1] : 1.0e-2);
         tikhonovPriorVarianceS.set(perBodyPriorVariance != null ? perBodyPriorVariance[4] : 1.0e-2);
         tikhonovPriorVarianceT.set(perBodyPriorVariance != null ? perBodyPriorVariance[7] : 1.0e-2);
         priorCovarianceDiag = new DMatrixRMaj(stateSize, stateSize);
         LogTools.info("InertialPhysicallyConsistentKalmanFilter: Tikhonov prior available (live YoVariables); enabled="
                       + tikhonovPriorEnabled.getValue());
      }
      else
      {
         tikhonovPriorEnabled = null;
         tikhonovPriorVarianceAlpha = null;
         tikhonovPriorVarianceD = null;
         tikhonovPriorVarianceS = null;
         tikhonovPriorVarianceT = null;
         priorCovarianceDiag = new DMatrixRMaj(0, 0);
      }
      priorInnovationCovariance = new DMatrixRMaj(stateSize, stateSize);
      priorInnovationCovarianceInv = new DMatrixRMaj(stateSize, stateSize);
      priorGain = new DMatrixRMaj(stateSize, stateSize);
      priorThetaFull = new DMatrixRMaj(stateSize, 1);
      priorStateContainer = new DMatrixRMaj(stateSize, 1);
      priorCovarianceContainer = new DMatrixRMaj(stateSize, stateSize);

      measurementJacobianBlock = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                                 RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      measurementJacobianContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies,
                                                     RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies);

      regressorBlock = new DMatrixRMaj(measurementCovariance.getNumRows(), RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);

      parameterThetaBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      kalmanGainBlockContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, measurementCovariance.getNumRows());

      measurement = new DMatrixRMaj(nDoFs, 1);

      DoubleProvider postProcessingAlpha = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt.getValue());
      filteredResidual = new AlphaFilteredYoMatrix("filteredResidual_", postProcessingAlpha, nDoFs, 1, parameters.getMeasurementNames(), null, registry);

      setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
   }

   /** For inertial parameters, the process model is the identity mapping -- we assume that the parameters are constant. */
   @Override
   protected DMatrixRMaj processModel(DMatrixRMaj parametersToEstimate)
   {
      return parametersToEstimate;
   }

   /**
    * For inertial parameters, the measurement model is the sum of:
    * <li> the regressor contribution from the parameters that are being estimated
    * <li> the regressor contribution from the nominal parameters
    * <li> the contribution from the contact wrenches mapped through the contact Jacobians
    */
   @Override
   protected DMatrixRMaj measurementModel(DMatrixRMaj parametersForEstimate)
   {
      // Torque from inverse dynamics on nominal model
      measurement.set(torqueFromNominal);

      // Torque from regressor on estimated parameters -- we do this bodywise because the physically-consistent parameterization is a bodywise mapping
      for (int i = 0; i < nBodies; ++i)
      {
         // Pack regressor block
         CommonOps_DDRM.extract(regressor, 0, i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, regressorBlock);
         CommonOps_DDRM.multAdd(regressorBlock, inertialParameters.get(i).getParameterVectorPiBasis(), measurement);
      }

      // Torque from contact wrenches
      for (RobotSide side : RobotSide.values)
      {
         // NOTE: the minus for the contact wrench contribution
         CommonOps_DDRM.multAddTransA(-1.0, contactJacobians.get(side), contactWrenches.get(side), measurement);
      }

      // Torque from bias
      CommonOps_DDRM.addEquals(measurement, torqueFromBias);

      return measurement;
   }

   /** For inertial parameters, the process Jacobian is the identity matrix. */
   @Override
   protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousParametersToEstimate)
   {
      return IDENTITY;
   }

   /** The measurement Jacobian for the physically-consistent parameterization is the regressor multiplied by the mapping Jacobian. */
   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      for (int i = 0; i < nBodies; ++i)
      {
         // Set the relevant block of the process jacobian (dispatches on absolute Eq. (1) vs generalized Eq. (11))
         inertialParameters.get(i).packThetaToPiBasisJacobian(measurementJacobianBlock);
         MatrixMissingTools.setMatrixBlock(measurementJacobianContainer,
                                           i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           measurementJacobianBlock,
                                           0,
                                           0,
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           1.0);
      }

      // Jacobian is Y * G(theta)
      CommonOps_DDRM.mult(regressor, measurementJacobianContainer, measurementJacobian);
      return measurementJacobian;
   }

   /**
    * The default update step of the extended Kalman filter is overwritten to account for the physically-consistent parameterization, which can only be
    * performed bodywise instead of the usual linear algebra approach of one big matrix.
    */
   @Override
   public void updateStep()
   {
      calculateKalmanGain();

      // Here, we deviate from the superclass and add the Kalman gain on bodywise, because we only have access to the Jacobian of the measurement model
      // in that way
      for (int i = 0; i < nBodies; ++i)
      {
         // For each body, get the Kalman gain block corresponding to it, and update the theta basis parameter vector with the residual
         MatrixMissingTools.setMatrixBlock(kalmanGainBlockContainer,
                                           0,
                                           0,
                                           kalmanGain,
                                           i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           0,
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           measurementCovariance.getNumRows(),
                                           1.0);
         parameterThetaBasisContainer.set(inertialParameters.get(i).getParameterVectorThetaBasis());
         CommonOps_DDRM.multAdd(kalmanGainBlockContainer, getMeasurementResidual(), parameterThetaBasisContainer);

         // Update inertial parameters
         inertialParameters.get(i).setParameterVectorThetaBasis(parameterThetaBasisContainer);
         inertialParameters.get(i).update();

         // Pack updated pi basis into state for the next iteration
         MatrixMissingTools.setMatrixRows(updatedState,
                                          i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                          inertialParameters.get(i).getParameterVectorPiBasis(),
                                          0,
                                          RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      }
      calculateUpdatedCovariance();

      state.set(updatedState);
      covariance.set(updatedCovariance);
   }

   /** Before the update step, low-pass filter the measurement residual to reduce the effect of contact impulses on acceleration, wrench, and joint torques. */
   @Override
   public void preUpdateHook()
   {
      filter(getMeasurementResidual(), filteredResidual);
      getMeasurementResidual().set(filteredResidual);
   }

   /** Post solve, optionally apply the Tikhonov prior toward nominal, then update the watchers. */
   @Override
   public void postSolveHook()
   {
      if (priorAvailable && tikhonovPriorEnabled.getValue())
         applyTikhonovPrior();
      updateWatchers();
   }

   /**
    * Tikhonov soft-constraint pseudo-measurement (z=0, H=I, covariance R_prior) pulling the estimate toward
    * theta=0 (nominal, in the generalized parameterization). The EKF covariance {@code covariance} is theta-space
    * and the persistent theta lives in {@code inertialParameters}, so the standard pseudo-measurement update
    * S = P + R_prior, K = P S^-1, theta <- theta - K theta, P <- (I-K) P is applied directly on the gathered
    * theta vector. In strongly-observed directions P << R_prior so K ~ 0 (mass untouched); in weakly-observed
    * directions P >> R_prior so K ~ I and theta is pulled toward 0 (CoM/inertia held near nominal). The updated
    * theta is scattered back to the per-body parameters (and the vestigial pi-basis state refreshed).
    */
   private void applyTikhonovPrior()
   {
      // Rebuild R_prior from the (live-tunable) per-parameter variances. Per-body diagonal pattern, repeated:
      // index 0 = alpha (mass), 1..3 = d (inertia diag), 4..6 = s (inertia off-diag), 7..9 = t (first moment).
      double varAlpha = tikhonovPriorVarianceAlpha.getValue();
      double varD = tikhonovPriorVarianceD.getValue();
      double varS = tikhonovPriorVarianceS.getValue();
      double varT = tikhonovPriorVarianceT.getValue();
      priorCovarianceDiag.zero();
      for (int b = 0; b < nBodies; ++b)
      {
         int base = b * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
         priorCovarianceDiag.set(base, base, varAlpha);
         for (int p = 1; p <= 3; ++p)
            priorCovarianceDiag.set(base + p, base + p, varD);
         for (int p = 4; p <= 6; ++p)
            priorCovarianceDiag.set(base + p, base + p, varS);
         for (int p = 7; p <= 9; ++p)
            priorCovarianceDiag.set(base + p, base + p, varT);
      }

      for (int i = 0; i < nBodies; ++i)
         MatrixMissingTools.setMatrixRows(priorThetaFull,
                                          i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                          inertialParameters.get(i).getParameterVectorThetaBasis(),
                                          0,
                                          RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);

      // S = P + R_prior ; K = P S^-1
      CommonOps_DDRM.add(covariance, priorCovarianceDiag, priorInnovationCovariance);
      if (!CommonOps_DDRM.invert(priorInnovationCovariance, priorInnovationCovarianceInv))
         return; // singular -- skip this tick rather than corrupt the estimate
      CommonOps_DDRM.mult(covariance, priorInnovationCovarianceInv, priorGain);

      // theta <- theta - K theta
      CommonOps_DDRM.mult(priorGain, priorThetaFull, priorStateContainer);
      CommonOps_DDRM.subtractEquals(priorThetaFull, priorStateContainer);

      // P <- P - K P   (reads covariance before it is overwritten)
      CommonOps_DDRM.mult(priorGain, covariance, priorCovarianceContainer);
      CommonOps_DDRM.subtractEquals(covariance, priorCovarianceContainer);

      // scatter theta back, refresh pi basis (drives the watchers and the body inertia consumed downstream)
      for (int i = 0; i < nBodies; ++i)
      {
         CommonOps_DDRM.extract(priorThetaFull,
                                i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                (i + 1) * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                0,
                                1,
                                parameterThetaBasisContainer,
                                0,
                                0);
         inertialParameters.get(i).setParameterVectorThetaBasis(parameterThetaBasisContainer);
         inertialParameters.get(i).update();
         MatrixMissingTools.setMatrixRows(state,
                                          i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                          inertialParameters.get(i).getParameterVectorPiBasis(),
                                          0,
                                          RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      }
   }

   @Override
   public void setRegressor(DMatrix regressor)
   {
      this.regressor.set(regressor);
   }

   @Override
   public void setTorqueFromNominal(DMatrix torqueFromNominal)
   {
      this.torqueFromNominal.set(torqueFromNominal);
   }

   @Override
   public void setTorqueFromBias(DMatrix torqueFromBias)
   {
      this.torqueFromBias.set(torqueFromBias);
   }

   @Override
   public void setContactJacobians(SideDependentList<DMatrixRMaj> jacobians)
   {
      for (RobotSide side : RobotSide.values)
         contactJacobians.get(side).set(jacobians.get(side));
   }

   @Override
   public void setContactWrenches(SideDependentList<DMatrixRMaj> wrenches)
   {
      for (RobotSide side : RobotSide.values)
         contactWrenches.get(side).set(wrenches.get(side));
   }

   private void filter(DMatrix matrixToFilter, AlphaFilteredYoMatrix filterContainer)
   {
      filterContainer.setAndSolve(matrixToFilter);
   }

   private void updateWatchers()
   {
      for (int i = 0; i < nBodies; ++i)
      {
         inertialParametersPiBasisWatchers.get(i).set(inertialParameters.get(i).getParameterVectorPiBasis());
         inertialParametersThetaBasisWatchers.get(i).set(inertialParameters.get(i).getParameterVectorThetaBasis());
      }
   }
}
