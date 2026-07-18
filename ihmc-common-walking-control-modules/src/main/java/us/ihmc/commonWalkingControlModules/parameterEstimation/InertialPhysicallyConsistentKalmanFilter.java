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
   private final YoDouble tikhonovPriorVarianceT;     // first moment / CoM t1,t2 (tight -> held near nominal)
   private final YoDouble tikhonovPriorVarianceT3;    // t3 (down-axis CoM) override; index 9, defaults to varT
   /**
    * Per-body runtime multiplier on that body's prior variances, default 1.0. Since a SMALLER variance means a
    * STRONGER pull toward nominal, scaling a body's variances DOWN pins its estimate at nominal, and scaling them UP
    * removes the pull (freeze in place). This is the knob an operator-intent gate drives: a non-adapting limb gets a
    * small scale (pinned to nominal); a limb whose grasp signal dropped mid-carry gets a large scale (held in place,
    * combined with a low process-noise scale). Indexed by body (same order as {@link #inertialParameters}).
    */
   private final YoDouble[] priorBodyScale;
   private final DMatrixRMaj priorCovarianceDiag;   // R_prior (full state size, block-diagonal per body)
   private final DMatrixRMaj priorInnovationCovariance; // S = P + R_prior
   private final DMatrixRMaj priorInnovationCovarianceInv;
   private final DMatrixRMaj priorGain;             // K = P S^-1
   private final DMatrixRMaj priorThetaFull;        // gathered theta of all bodies (the EKF covariance is theta-space)
   private final DMatrixRMaj priorStateContainer;
   private final DMatrixRMaj priorCovarianceContainer;
   // CoM-load coupling: per-body payload attachment point cp (inertia frame) or null; the prior's first-moment
   // target becomes cp*(1-exp(-2*alpha)) instead of zero, tying the CoM to the observable mass. See
   // InertialEstimationParameters.getCoMLoadCouplingPoint.
   private final double[][] comCouplingPoints;
   private boolean hasCoMCoupling = false;
   private final DMatrixRMaj priorTargetTheta;      // prior pseudo-measurement target (zero, except coupled first moments)
   private final DMatrixRMaj priorDeltaContainer;   // theta - target

   private final DMatrixRMaj torqueFromNominal;
   private final DMatrixRMaj torqueFromBias;

   private final DMatrixRMaj regressor;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   // Hand/nub contacts (grasp rejection): a second contact channel, subtracted exactly like the feet. Only the
   // INTERNAL (squeeze / grasp-map null-space) component of the nub wrench is fed here, so it is removed from the
   // measurement while the external gravitational share stays in the regressor. Zero unless grasp rejection is on.
   private final SideDependentList<DMatrixRMaj> handContactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> handContactWrenches = new SideDependentList<>();

   private int nBodies;
   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final DMatrixRMaj measurementJacobianBlock;
   private final DMatrixRMaj measurementJacobianContainer;

   private final DMatrixRMaj regressorBlock;

   private final DMatrixRMaj parameterThetaBasisContainer;
   private final DMatrixRMaj parameterPiBasisContainer;

   private final DMatrixRMaj kalmanGainBlockContainer;

   private final DMatrixRMaj measurement;

   private final AlphaFilteredYoMatrix filteredResidual;
   /**
    * Set by {@link #reset()} to clear the residual filter's memory. {@link AlphaFilteredYoMatrix} has no reset, but it
    * asks us for its alpha every tick, so handing it alpha = 0 for one tick makes its own update
    * ({@code filtered = alpha * previous + (1 - alpha) * current}) overwrite the remembered (pre-reset) output with the
    * current value. Cleared again as soon as that tick's filter call has happened.
    */
   private boolean clearResidualFilterMemory = false;

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
         handContactJacobians.put(side, new DMatrixRMaj(Wrench.SIZE, nDoFs));
         handContactWrenches.put(side, new DMatrixRMaj(Wrench.SIZE, 1));
      }

      java.util.List<double[]> comCouplingList = new java.util.ArrayList<>();
      java.util.List<String> estimatedBodyNames = new java.util.ArrayList<>();
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
            comCouplingList.add(parameters.getCoMLoadCouplingPoint(modelBodies[i].getName()));
            inertialParametersPiBasisWatchers.add(new YoMatrix("pi_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForPiBasis(), null, registry));
            inertialParametersThetaBasisWatchers.add(new YoMatrix("theta_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForThetaBasis(), null, registry));
            estimatedBodyNames.add(modelBodies[i].getName());
            nBodies++;
         }
      }

      comCouplingPoints = comCouplingList.toArray(new double[0][]);
      for (double[] cp : comCouplingPoints)
         if (cp != null)
            hasCoMCoupling = true;
      if (hasCoMCoupling)
         LogTools.info("InertialPhysicallyConsistentKalmanFilter: CoM-load coupling ON (first-moment prior tracks cp*(1-e^-2a)).");

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
         tikhonovPriorVarianceT3 = new YoDouble("tikhonovPriorVarianceT3", registry);
         // index map within a body: 0=alpha, 1..3=d, 4..6=s, 7..9=t
         tikhonovPriorVarianceAlpha.set(perBodyPriorVariance != null ? perBodyPriorVariance[0] : 1.0e6);
         tikhonovPriorVarianceD.set(perBodyPriorVariance != null ? perBodyPriorVariance[1] : 1.0e-2);
         tikhonovPriorVarianceS.set(perBodyPriorVariance != null ? perBodyPriorVariance[4] : 1.0e-2);
         tikhonovPriorVarianceT.set(perBodyPriorVariance != null ? perBodyPriorVariance[7] : 1.0e-2);
         tikhonovPriorVarianceT3.set(perBodyPriorVariance != null ? perBodyPriorVariance[9] : 1.0e-2);
         priorCovarianceDiag = new DMatrixRMaj(stateSize, stateSize);
         priorBodyScale = new YoDouble[nBodies];
         for (int b = 0; b < nBodies; ++b)
         {
            priorBodyScale[b] = new YoDouble(estimatedBodyNames.get(b) + "_priorScale", registry);
            priorBodyScale[b].set(1.0);
         }
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
         tikhonovPriorVarianceT3 = null;
         priorBodyScale = null;
         priorCovarianceDiag = new DMatrixRMaj(0, 0);
      }
      priorInnovationCovariance = new DMatrixRMaj(stateSize, stateSize);
      priorInnovationCovarianceInv = new DMatrixRMaj(stateSize, stateSize);
      priorGain = new DMatrixRMaj(stateSize, stateSize);
      priorThetaFull = new DMatrixRMaj(stateSize, 1);
      priorStateContainer = new DMatrixRMaj(stateSize, 1);
      priorCovarianceContainer = new DMatrixRMaj(stateSize, stateSize);
      priorTargetTheta = new DMatrixRMaj(stateSize, 1);
      priorDeltaContainer = new DMatrixRMaj(stateSize, 1);

      measurementJacobianBlock = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                                 RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      measurementJacobianContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies,
                                                     RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies);

      regressorBlock = new DMatrixRMaj(measurementCovariance.getNumRows(), RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);

      parameterThetaBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      parameterPiBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      kalmanGainBlockContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, measurementCovariance.getNumRows());

      measurement = new DMatrixRMaj(nDoFs, 1);

      DoubleProvider postProcessingAlpha = () -> clearResidualFilterMemory ? 0.0
            : AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt.getValue());
      filteredResidual = new AlphaFilteredYoMatrix("filteredResidual_", postProcessingAlpha, nDoFs, 1, parameters.getMeasurementNames(), null, registry);

      setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
   }

   /**
    * Re-seeds the filter to the nominal model: on top of the base class re-seeding the state and covariance, the
    * per-body parameters -- which are the authoritative state of this filter, the base class's Pi-basis state being
    * only a re-packed copy -- are put back to nominal, and the residual filter's memory is cleared.
    * <p>
    * In the generalized parameterization theta = 0 is exactly the nominal, so the deviation is simply zeroed. In the
    * absolute parameterization there is no such fixed point, so each body is re-seeded from the URDF Pi-basis values
    * that the base class just restored into {@code state}.
    * </p>
    */
   @Override
   public void reset()
   {
      super.reset();

      for (int i = 0; i < nBodies; ++i)
      {
         RigidBodyInertialParameters bodyParameters = inertialParameters.get(i);
         if (bodyParameters.isGeneralized())
         {
            parameterThetaBasisContainer.zero();
            bodyParameters.setParameterVectorThetaBasis(parameterThetaBasisContainer);
         }
         else
         {
            CommonOps_DDRM.extract(state,
                                   i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                   (i + 1) * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                   0,
                                   1,
                                   parameterPiBasisContainer,
                                   0,
                                   0);
            bodyParameters.setParameterVectorPiBasis(parameterPiBasisContainer);
         }
         bodyParameters.update();

         // Keep the base class's Pi-basis state in lockstep with the per-body parameters, as the update step does.
         MatrixMissingTools.setMatrixRows(state,
                                          i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                          bodyParameters.getParameterVectorPiBasis(),
                                          0,
                                          RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      }

      filteredResidual.zero();
      clearResidualFilterMemory = true;
      updateWatchers();
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

      // Torque from hand/nub contact wrenches (internal grasp component only; zero unless grasp rejection is on)
      for (RobotSide side : RobotSide.values)
      {
         CommonOps_DDRM.multAddTransA(-1.0, handContactJacobians.get(side), handContactWrenches.get(side), measurement);
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
      // The filter has now consumed the one zeroed alpha, so its memory holds the post-reset residual, not the
      // pre-reset one. Back to the tuned alpha from here on.
      clearResidualFilterMemory = false;
      getMeasurementResidual().set(filteredResidual);
   }

   /**
    * Post solve, optionally apply the Tikhonov prior toward nominal, then update the watchers.
    * <p>
    * The prior mutates theta, so it must be skipped while the parameters are held -- otherwise the hold leaks and
    * the residual being averaged for the bias would no longer correspond to a fixed parameter value.
    * </p>
    */
   @Override
   public void postSolveHook()
   {
      if (priorAvailable && tikhonovPriorEnabled.getValue() && !isParametersHeld())
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
      double varT3 = tikhonovPriorVarianceT3.getValue();
      priorCovarianceDiag.zero();
      for (int b = 0; b < nBodies; ++b)
      {
         int base = b * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
         // Per-body scale on the variances: <1 tightens (pins to nominal), >1 loosens (removes the pull). This is
         // how an idle limb is pinned at nominal while an actively-adapting limb keeps the base (loose) prior.
         double sb = priorBodyScale[b].getValue();
         priorCovarianceDiag.set(base, base, sb * varAlpha);
         for (int p = 1; p <= 3; ++p)
            priorCovarianceDiag.set(base + p, base + p, sb * varD);
         for (int p = 4; p <= 6; ++p)
            priorCovarianceDiag.set(base + p, base + p, sb * varS);
         priorCovarianceDiag.set(base + 7, base + 7, sb * varT);
         priorCovarianceDiag.set(base + 8, base + 8, sb * varT);
         priorCovarianceDiag.set(base + 9, base + 9, sb * varT3);
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

      // Prior target: 0 everywhere, EXCEPT the first-moment (t) channels of coupled bodies, which track
      // cp*(1 - exp(-2*alpha)) -- the combined CoM of "nominal link + point payload at cp" as the observable
      // mass channel alpha grows. This ties the degenerate CoM to the mass, so the observed first moment
      // m*c resolves to a unique (m, c). With no coupling the target is zero and this reduces to theta -= K*theta.
      priorTargetTheta.zero();
      if (hasCoMCoupling)
      {
         for (int b = 0; b < nBodies; ++b)
         {
            double[] cp = comCouplingPoints[b];
            if (cp == null)
               continue;
            int base = b * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
            double alpha = priorThetaFull.get(base, 0);
            double factor = Math.max(0.0, 1.0 - Math.exp(-2.0 * alpha));
            priorTargetTheta.set(base + 7, 0, cp[0] * factor);
            priorTargetTheta.set(base + 8, 0, cp[1] * factor);
            priorTargetTheta.set(base + 9, 0, cp[2] * factor);
         }
      }

      // theta <- theta - K (theta - target)
      CommonOps_DDRM.subtract(priorThetaFull, priorTargetTheta, priorDeltaContainer);
      CommonOps_DDRM.mult(priorGain, priorDeltaContainer, priorStateContainer);
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

   /**
    * Set the runtime prior-variance multiplier for estimated body {@code b} (see {@link #priorBodyScale}). No-op if
    * the generalized prior is unavailable. {@code scale < 1} pins the body to nominal; {@code scale > 1} removes the
    * pull (freeze in place, when combined with a low process-noise scale).
    */
   @Override
   public void setPriorScaleForBody(int bodyIndex, double scale)
   {
      if (priorBodyScale != null && bodyIndex >= 0 && bodyIndex < priorBodyScale.length)
         priorBodyScale[bodyIndex].set(scale);
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

   @Override
   public void setHandContactJacobians(SideDependentList<DMatrixRMaj> jacobians)
   {
      for (RobotSide side : RobotSide.values)
         handContactJacobians.get(side).set(jacobians.get(side));
   }

   @Override
   public void setHandContactWrenches(SideDependentList<DMatrixRMaj> wrenches)
   {
      for (RobotSide side : RobotSide.values)
         handContactWrenches.get(side).set(wrenches.get(side));
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
