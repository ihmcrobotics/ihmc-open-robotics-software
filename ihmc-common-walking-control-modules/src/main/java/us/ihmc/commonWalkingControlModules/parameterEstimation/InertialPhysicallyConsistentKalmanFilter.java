package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
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
   // Excitation diagnostics (logging only): diag of the theta-space covariance P, and an exponentially windowed
   // information matrix Omega = sum H_b^T R^-1 H_b per body (time constant INFORMATION_WINDOW), its diagonal,
   // eigenvalues (ascending) and condition number.
   private static final double INFORMATION_WINDOW = 2.0;
   // Optional geometric prior (Cho, Lee and Shin, ICRA 2024; Lee, Wensing and Park, T-RO 2020): instead of the diagonal
   // per-channel variances, use R_prior = (w H0)^-1 per body, where H0 is the Hessian at theta = 0 of the log-det
   // (Bregman) divergence D(J(theta) || J_nominal) between pseudo-inertias. Coordinate-invariant: mass, first moment
   // and rotational inertia are weighted by their effect on the pseudo-inertia rather than by hand-picked variances.
   // -DinertialEstimator.priorMetric=DIAG (default) | LOGDET, -DinertialEstimator.logDetPriorWeight (default 10).
   private static final boolean LOG_DET_PRIOR = "LOGDET".equalsIgnoreCase(System.getProperty("inertialEstimator.priorMetric", "DIAG"));
   private static final double LOG_DET_PRIOR_WEIGHT = Double.parseDouble(System.getProperty("inertialEstimator.logDetPriorWeight", "10.0"));
   // ALL: geometric prior on every channel. INERTIA: geometric metric restricted to the rotational-inertia channels
   // (d, s), leaving mass (alpha) and first moment (t) unregularized, as in the paper's inertia-only prior.
   private static final boolean LOG_DET_PRIOR_INERTIA_ONLY = "INERTIA".equalsIgnoreCase(System.getProperty("inertialEstimator.logDetPriorSubspace", "INERTIA"));
   private final List<DMatrixRMaj> logDetPriorCovarianceBlocks = new ArrayList<>();
   private final List<YoMatrix> covarianceDiagonalWatchers = new ArrayList<>();
   private final List<YoMatrix> informationDiagonalWatchers = new ArrayList<>();
   private final List<YoMatrix> informationEigenvalueWatchers = new ArrayList<>();
   private final List<YoDouble> informationConditionWatchers = new ArrayList<>();
   private final List<DMatrixRMaj> windowedInformation = new ArrayList<>();
   private final DMatrixRMaj informationJacobianBlock = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj informationIncrement = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                                                    RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
   private final DMatrixRMaj informationScratch = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                                                  RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
   private final EigenDecomposition_F64<DMatrixRMaj> informationEigenDecomposition =
         DecompositionFactory_DDRM.eig(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, false, true);
   private final double[] informationEigenvalues = new double[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
   private final DoubleProvider diagnosticsDt;

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
      diagnosticsDt = dt;

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
            String[] eigenNames = new String[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
            for (int e = 0; e < eigenNames.length; e++)
               eigenNames[e] = "e" + e;
            covarianceDiagonalWatchers.add(new YoMatrix("ekfPdiag_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForThetaBasis(), null, registry));
            informationDiagonalWatchers.add(new YoMatrix("ekfInfoDiag_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForThetaBasis(), null, registry));
            informationEigenvalueWatchers.add(new YoMatrix("ekfInfoEig_" + modelBodies[i].getName() + "_", 10, 1, eigenNames, null, registry));
            informationConditionWatchers.add(new YoDouble("ekfInfoCond_" + modelBodies[i].getName(), registry));
            windowedInformation.add(new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY));
            estimatedBodyNames.add(modelBodies[i].getName());
            nBodies++;
         }
      }

      comCouplingPoints = comCouplingList.toArray(new double[0][]);
      if (LOG_DET_PRIOR && parameters.useGeneralizedPhysicalConsistency())
      {
         for (RigidBodyInertialParameters bodyParameters : inertialParameters)
            logDetPriorCovarianceBlocks.add(computeLogDetPriorCovariance(bodyParameters, LOG_DET_PRIOR_WEIGHT));
         LogTools.info("InertialPhysicallyConsistentKalmanFilter: log-det (geometric) prior, weight " + LOG_DET_PRIOR_WEIGHT);
      }
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

   @Override
   public void applySetpoint(double massScale, double comShiftX, double comShiftY, double comShiftZ, double covarianceScale)
   {
      double[] shift = {comShiftX, comShiftY, comShiftZ};
      for (int i = 0; i < nBodies; ++i)
      {
         RigidBodyInertialParameters bodyParameters = inertialParameters.get(i);
         DMatrixRMaj pi = bodyParameters.getParameterVectorPiBasis();
         double m = pi.get(0, 0);
         if (!(m > 0.0))
            continue;
         double[] c = {pi.get(1, 0) / m, pi.get(2, 0) / m, pi.get(3, 0) / m};
         // Rotational inertia about the origin, Pi ordering Ixx, Ixy, Ixz, Iyy, Iyz, Izz.
         double[][] inertia = {{pi.get(4, 0), pi.get(5, 0), pi.get(6, 0)},
                               {pi.get(5, 0), pi.get(7, 0), pi.get(8, 0)},
                               {pi.get(6, 0), pi.get(8, 0), pi.get(9, 0)}};
         double newMass = massScale * m;
         double[] newCoM = {c[0] + shift[0], c[1] + shift[1], c[2] + shift[2]};
         double c2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
         double n2 = newCoM[0] * newCoM[0] + newCoM[1] * newCoM[1] + newCoM[2] * newCoM[2];
         double[][] newInertia = new double[3][3];
         for (int r = 0; r < 3; r++)
         {
            for (int k = 0; k < 3; k++)
            {
               double delta = r == k ? 1.0 : 0.0;
               double aboutCoM = inertia[r][k] - m * (c2 * delta - c[r] * c[k]);
               newInertia[r][k] = massScale * aboutCoM + newMass * (n2 * delta - newCoM[r] * newCoM[k]);
            }
         }
         parameterPiBasisContainer.set(0, 0, newMass);
         for (int r = 0; r < 3; r++)
            parameterPiBasisContainer.set(1 + r, 0, newMass * newCoM[r]);
         parameterPiBasisContainer.set(4, 0, newInertia[0][0]);
         parameterPiBasisContainer.set(5, 0, newInertia[0][1]);
         parameterPiBasisContainer.set(6, 0, newInertia[0][2]);
         parameterPiBasisContainer.set(7, 0, newInertia[1][1]);
         parameterPiBasisContainer.set(8, 0, newInertia[1][2]);
         parameterPiBasisContainer.set(9, 0, newInertia[2][2]);
         bodyParameters.setParameterVectorPiBasis(parameterPiBasisContainer);
         bodyParameters.update();
         MatrixMissingTools.setMatrixRows(state,
                                          i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                          bodyParameters.getParameterVectorPiBasis(),
                                          0,
                                          RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      }
      CommonOps_DDRM.scale(covarianceScale, covariance);
      filteredResidual.zero();
      clearResidualFilterMemory = true;
      updateWatchers();
   }

   @Override
   public void setBodyState(int bodyIndex, DMatrix pi, boolean resetCovariance)
   {
      if (bodyIndex < 0 || bodyIndex >= nBodies)
         return;
      RigidBodyInertialParameters bodyParameters = inertialParameters.get(bodyIndex);
      for (int r = 0; r < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; r++)
         parameterPiBasisContainer.set(r, 0, pi.get(r, 0));
      bodyParameters.setParameterVectorPiBasis(parameterPiBasisContainer);
      bodyParameters.update();
      MatrixMissingTools.setMatrixRows(state,
                                       bodyIndex * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                       bodyParameters.getParameterVectorPiBasis(),
                                       0,
                                       RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      if (resetCovariance)
         covariance.set(initialCovariance);
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
      updateExcitationDiagnostics();
   }

   /** Logging only: diag(P), and the windowed information matrix of each body's theta block with its spectrum. */
   private void updateExcitationDiagnostics()
   {
      int n = RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
      double decay = Math.exp(-diagnosticsDt.getValue() / INFORMATION_WINDOW);
      int nMeasurements = measurementJacobian.getNumRows();
      informationJacobianBlock.reshape(nMeasurements, n);
      for (int b = 0; b < nBodies; ++b)
      {
         YoMatrix pDiagonal = covarianceDiagonalWatchers.get(b);
         for (int j = 0; j < n; j++)
            pDiagonal.set(j, 0, covariance.get(b * n + j, b * n + j));

         // H_b^T R^-1 H_b with R diagonal.
         CommonOps_DDRM.extract(measurementJacobian, 0, nMeasurements, b * n, (b + 1) * n, informationJacobianBlock, 0, 0);
         informationIncrement.zero();
         for (int r = 0; r < nMeasurements; r++)
         {
            double rInverse = 1.0 / measurementCovariance.get(r, r);
            for (int j = 0; j < n; j++)
            {
               double hj = informationJacobianBlock.get(r, j) * rInverse;
               if (hj == 0.0)
                  continue;
               for (int k = 0; k < n; k++)
                  informationIncrement.add(j, k, hj * informationJacobianBlock.get(r, k));
            }
         }
         DMatrixRMaj omega = windowedInformation.get(b);
         CommonOps_DDRM.scale(decay, omega);
         CommonOps_DDRM.addEquals(omega, informationIncrement);

         YoMatrix infoDiagonal = informationDiagonalWatchers.get(b);
         for (int j = 0; j < n; j++)
            infoDiagonal.set(j, 0, omega.get(j, j));

         informationScratch.set(omega);
         if (informationEigenDecomposition.decompose(informationScratch))
         {
            for (int e = 0; e < n; e++)
               informationEigenvalues[e] = informationEigenDecomposition.getEigenvalue(e).getReal();
            java.util.Arrays.sort(informationEigenvalues);
            YoMatrix eigen = informationEigenvalueWatchers.get(b);
            for (int e = 0; e < n; e++)
               eigen.set(e, 0, informationEigenvalues[e]);
            double smallest = Math.max(informationEigenvalues[0], 1.0e-300);
            informationConditionWatchers.get(b).set(informationEigenvalues[n - 1] / smallest);
         }
      }
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
      if (!logDetPriorCovarianceBlocks.isEmpty())
      {
         priorCovarianceDiag.zero();
         for (int b = 0; b < nBodies; ++b)
         {
            int base = b * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
            MatrixMissingTools.setMatrixBlock(priorCovarianceDiag, base, base, logDetPriorCovarianceBlocks.get(b), 0, 0,
                                              RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                              RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                              priorBodyScale[b].getValue());
         }
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

   /** (w H0)^-1, with H0 the Hessian of the log-det divergence to the nominal at theta = 0 (central differences). */
   private static DMatrixRMaj computeLogDetPriorCovariance(RigidBodyInertialParameters bodyParameters, double weight)
   {
      int n = RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
      DMatrixRMaj nominalPi = new DMatrixRMaj(bodyParameters.getParameterVectorPiBasis());
      DMatrixRMaj nominalTheta = new DMatrixRMaj(n, 1);
      RigidBodyInertialParameters.fromPiBasisToThetaBasis(nominalPi, nominalTheta);
      DMatrixRMaj nominalPseudoInverse = pseudoInertia(nominalPi);
      CommonOps_DDRM.invert(nominalPseudoInverse);

      double h = 1.0e-3;
      DMatrixRMaj hessian = new DMatrixRMaj(n, n);
      double d0 = logDetDivergence(new DMatrixRMaj(n, 1), nominalTheta, nominalPseudoInverse);
      for (int j = 0; j < n; j++)
      {
         for (int k = j; k < n; k++)
         {
            double[] signs = {1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, -1.0};
            double[] values = new double[4];
            for (int c = 0; c < 4; c++)
            {
               DMatrixRMaj deviation = new DMatrixRMaj(n, 1);
               deviation.add(j, 0, signs[2 * c] * h);
               deviation.add(k, 0, signs[2 * c + 1] * h);
               values[c] = logDetDivergence(deviation, nominalTheta, nominalPseudoInverse) - d0;
            }
            double value = (values[0] - values[1] - values[2] + values[3]) / (4.0 * h * h);
            hessian.set(j, k, value);
            hessian.set(k, j, value);
         }
      }
      CommonOps_DDRM.scale(weight, hessian);
      DMatrixRMaj covarianceBlock = new DMatrixRMaj(n, n);
      if (LOG_DET_PRIOR_INERTIA_ONLY)
      {
         // Prior information only on the inertia channels 1..6; alpha and t get a negligible prior (variance 1e6).
         DMatrixRMaj inertiaInformation = CommonOps_DDRM.extract(hessian, 1, 7, 1, 7);
         DMatrixRMaj inertiaCovariance = new DMatrixRMaj(6, 6);
         if (!CommonOps_DDRM.invert(inertiaInformation, inertiaCovariance))
            throw new RuntimeException("Log-det prior inertia Hessian is singular");
         CommonOps_DDRM.insert(inertiaCovariance, covarianceBlock, 1, 1);
         covarianceBlock.set(0, 0, 1.0e6);
         for (int t = 7; t < n; t++)
            covarianceBlock.set(t, t, 1.0e6);
         return covarianceBlock;
      }
      if (!CommonOps_DDRM.invert(hessian, covarianceBlock))
         throw new RuntimeException("Log-det prior Hessian is singular");
      return covarianceBlock;
   }

   private static double logDetDivergence(DMatrixRMaj deviation, DMatrixRMaj nominalTheta, DMatrixRMaj nominalPseudoInverse)
   {
      int n = RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
      DMatrixRMaj composed = new DMatrixRMaj(n, 1);
      DMatrixRMaj pi = new DMatrixRMaj(n, 1);
      RigidBodyInertialParameters.composeThetaBasis(deviation, nominalTheta, composed);
      RigidBodyInertialParameters.fromThetaBasisToPiBasis(composed, pi);
      DMatrixRMaj product = new DMatrixRMaj(4, 4);
      CommonOps_DDRM.mult(nominalPseudoInverse, pseudoInertia(pi), product);
      return -Math.log(CommonOps_DDRM.det(product)) + CommonOps_DDRM.trace(product) - 4.0;
   }

   /** Pseudo-inertia [[0.5 tr(I) 1 - I, m c], [(m c)^T, m]] from a Pi-basis vector (I about the body-frame origin). */
   private static DMatrixRMaj pseudoInertia(DMatrixRMaj pi)
   {
      double ixx = pi.get(4, 0), ixy = pi.get(5, 0), ixz = pi.get(6, 0), iyy = pi.get(7, 0), iyz = pi.get(8, 0), izz = pi.get(9, 0);
      double halfTrace = 0.5 * (ixx + iyy + izz);
      DMatrixRMaj pseudo = new DMatrixRMaj(4, 4);
      pseudo.set(0, 0, halfTrace - ixx);
      pseudo.set(1, 1, halfTrace - iyy);
      pseudo.set(2, 2, halfTrace - izz);
      pseudo.set(0, 1, -ixy);
      pseudo.set(1, 0, -ixy);
      pseudo.set(0, 2, -ixz);
      pseudo.set(2, 0, -ixz);
      pseudo.set(1, 2, -iyz);
      pseudo.set(2, 1, -iyz);
      for (int r = 0; r < 3; r++)
      {
         pseudo.set(r, 3, pi.get(1 + r, 0));
         pseudo.set(3, r, pi.get(1 + r, 0));
      }
      pseudo.set(3, 3, pi.get(0, 0));
      return pseudo;
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
