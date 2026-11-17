package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoMatrix;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * An implementation of the {@link ExtendedKalmanFilter} for estimating inertial parameters of a robot, where the nonlinear features of the
 * extended Kalman filter are not actually used, making this a vanilla Kalman filter.
 * <p>
 * Features of the filter:
 * <li> The process model of the filter is how the inertial parameters are expected to change over time. As we cannot predict this, we consider the inertial
 * parameters to be constant and therefore process model is the identity mapping.
 * <li> The measurement of the filter is the torque contribution from the inertial parameters being estimated. We form a model of this by totaling up the other
 * torque contributions: the contribution from the current estimate of the inertial parameters, the contribution from the nominal inertial parameters that are
 * considered known and fixed, the contribution from the contact wrenches, and the contribution from a bias that approximates model mismatch.
 * </p>
 *
 * @author James Foster
 */
public class InertialKalmanFilter extends ExtendedKalmanFilter implements OnlineInertialEstimator
{
   private final DMatrixRMaj IDENTITY;

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

   private final DMatrixRMaj measurement;

   private final AlphaFilteredYoMatrix filteredResidual;
   /**
    * Set by {@link #reset()} to clear the residual filter's memory. {@link AlphaFilteredYoMatrix} has no reset, but it
    * asks us for its alpha every tick, so handing it alpha = 0 for one tick makes its own update
    * ({@code filtered = alpha * previous + (1 - alpha) * current}) overwrite the remembered (pre-reset) output with the
    * current value. Cleared again as soon as that tick's filter call has happened.
    */
   private boolean clearResidualFilterMemory = false;

   /** State constraint applied after every update; see {@link InertialEstimationParameters#getKalmanFilterConstraint()}. */
   private enum StateConstraint
   {
      NONE, MASS_CLAMP, PSD_PROJECT,
      /** Constrained MAP: projection onto the pseudo-inertia LMI in the filter's own P^-1 metric ({@link PseudoInertiaConstrainedProjection}). */
      LMI_MAP,
      /**
       * [TMECH] Log-determinant (entropic) MAP of Lee, Wensing and Park (T-RO 2020), output only: the filter recursion keeps
       * the unconstrained mean (the least-squares sufficient statistic), and every tick the estimate handed out is
       * argmin 1/2 |pi - mean|^2_{P^-1} + gamma D(J(pi) || J_nominal). See {@link PseudoInertiaConstrainedProjection#logDetMap}.
       */
      LOGDET_MAP
   }

   private static final int PI_SIZE = 10;
   /** Floor on each body's mass [kg] (MASS_CLAMP) and on the pseudo-inertia eigenvalues (PSD_PROJECT). */
   private static final double MASS_FLOOR = 0.05;
   private static final double EIGENVALUE_FLOOR = 1.0e-6;

   private final StateConstraint stateConstraint;
   /** Per-body pi-basis estimate watchers ("pi_<body>_*"), named as in the physically consistent EKF for like-for-like logging. */
   private final List<YoMatrix> piWatchers = new ArrayList<>();
   private final List<Integer> piWatcherOffsets = new ArrayList<>();
   private final DMatrixRMaj piBlock = new DMatrixRMaj(PI_SIZE, 1);
   private final DMatrixRMaj pseudoInertia = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj projectedPseudoInertia = new DMatrixRMaj(4, 4);
   private final EigenDecomposition_F64<DMatrixRMaj> eigenDecomposition = DecompositionFactory_DDRM.eig(4, true, true);
   // LMI_MAP: constrained-MAP projection in the P^-1 metric (eps = 1e-6 on the pseudo-inertia eigenvalues, duality gap 1e-6).
   private final PseudoInertiaConstrainedProjection lmiProjection = new PseudoInertiaConstrainedProjection(1.0e-6, 1.0e-6);
   private final DMatrixRMaj lmiW = new DMatrixRMaj(PI_SIZE, PI_SIZE);
   private final DMatrixRMaj lmiStart = new DMatrixRMaj(PI_SIZE, 1);
   private final DMatrixRMaj lmiResult = new DMatrixRMaj(PI_SIZE, 1);
   private DMatrixRMaj lmiLast = null;
   private us.ihmc.yoVariables.variable.YoDouble lmiDistance, lmiNewtonSteps;
   private us.ihmc.yoVariables.variable.YoBoolean lmiActive;
   // LOGDET_MAP: exact divergence (eps = 0); the unconstrained mean is restored before every prediction step.
   private final PseudoInertiaConstrainedProjection logDetMapSolver = new PseudoInertiaConstrainedProjection(0.0, 1.0e-6);
   private final DMatrixRMaj logDetJ0Inverse = new DMatrixRMaj(4, 4);
   private final double logDetMapWeight;
   private DMatrixRMaj unconstrainedMean = null;
   private DMatrixRMaj logDetMapLast = null;
   private us.ihmc.yoVariables.variable.YoDouble logDetMapDivergence, logDetMapNewtonSteps, logDetMapDistance;
   private us.ihmc.yoVariables.variable.YoBoolean logDetMapConverged;

   public InertialKalmanFilter(FullRobotModel model, InertialEstimationParameters parameters, DoubleProvider dt, YoRegistry parentRegistry)
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

      regressor = new DMatrixRMaj(nDoFs, partitionSizes[0]);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(Wrench.SIZE, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(Wrench.SIZE, 1));
         handContactJacobians.put(side, new DMatrixRMaj(Wrench.SIZE, nDoFs));
         handContactWrenches.put(side, new DMatrixRMaj(Wrench.SIZE, 1));
      }

      measurement = new DMatrixRMaj(nDoFs, 1);

      DoubleProvider postProcessingAlpha = () -> clearResidualFilterMemory ? 0.0
            : AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt.getValue());
      filteredResidual = new AlphaFilteredYoMatrix("filteredResidual_", postProcessingAlpha, nDoFs, 1, parameters.getMeasurementNames(), null, registry);

      setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());

      RigidBodyReadOnly[] modelBodies = model.getRootBody().subtreeArray();
      int offset = 0;
      for (int i = 0; i < modelBodies.length && i < basisSets.length; i++)
      {
         if (basisSets[i].size() == PI_SIZE)
         {
            piWatchers.add(new YoMatrix("pi_" + modelBodies[i].getName() + "_", PI_SIZE, 1, RigidBodyInertialParametersTools.getNamesForPiBasis(), null, registry));
            piWatcherOffsets.add(offset);
         }
         offset += basisSets[i].size();
      }

      stateConstraint = StateConstraint.valueOf(parameters.getKalmanFilterConstraint());
      if (stateConstraint == StateConstraint.LMI_MAP)
      {
         // registered at construction so the variables are in the session's buffer
         lmiDistance = new us.ihmc.yoVariables.variable.YoDouble("lmiMapDistance", registry);
         lmiNewtonSteps = new us.ihmc.yoVariables.variable.YoDouble("lmiMapNewtonSteps", registry);
         lmiActive = new us.ihmc.yoVariables.variable.YoBoolean("lmiMapActive", registry);
      }
      logDetMapWeight = parameters.getKalmanFilterLogDetWeight();
      if (stateConstraint == StateConstraint.LOGDET_MAP)
      {
         logDetMapDivergence = new us.ihmc.yoVariables.variable.YoDouble("logDetMapDivergence", registry);
         logDetMapNewtonSteps = new us.ihmc.yoVariables.variable.YoDouble("logDetMapNewtonSteps", registry);
         logDetMapDistance = new us.ihmc.yoVariables.variable.YoDouble("logDetMapDistance", registry);
         logDetMapConverged = new us.ihmc.yoVariables.variable.YoBoolean("logDetMapConverged", registry);
      }
      if (stateConstraint != StateConstraint.NONE && getState().getNumRows() % PI_SIZE != 0)
         throw new IllegalArgumentException("KF state constraint " + stateConstraint + " needs the full 10-parameter pi basis per body");
   }

   /** On top of the base class re-seeding the state and covariance, clear the residual filter's memory. */
   @Override
   public void reset()
   {
      super.reset();
      filteredResidual.zero();
      clearResidualFilterMemory = true;
      if (unconstrainedMean != null)
         unconstrainedMean.set(state);
      if (logDetMapLast != null)
         logDetMapLast.set(state);
   }

   /** [TMECH] Absolute Pi-basis state, so the body is written straight into rows 10 i .. 10 i + 9 (full basis only). */
   @Override
   public void setBodyState(int bodyIndex, DMatrix pi, boolean resetCovariance)
   {
      if (state.getNumRows() % PI_SIZE != 0 || bodyIndex < 0 || (bodyIndex + 1) * PI_SIZE > state.getNumRows())
         return;
      for (int r = 0; r < PI_SIZE; r++)
      {
         state.set(bodyIndex * PI_SIZE + r, 0, pi.get(r, 0));
         if (unconstrainedMean != null)
            unconstrainedMean.set(bodyIndex * PI_SIZE + r, 0, pi.get(r, 0));
         if (logDetMapLast != null)
            logDetMapLast.set(bodyIndex * PI_SIZE + r, 0, pi.get(r, 0));
      }
      if (resetCovariance)
         covariance.set(initialCovariance);
      filteredResidual.zero();
      clearResidualFilterMemory = true;
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

      // Torque from regressor on estimated parameters
      CommonOps_DDRM.multAdd(regressor, parametersForEstimate, measurement);

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

   /** For inertial parameters, the measurement Jacobian is the regressor corresponding to the parameters one is estimating. */
   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedParametersToEstimate)
   {
      return regressor;
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
    * Constrained KF by state projection (Simon 2010): after the update, project each body's pi block onto the
    * constraint set. The covariance is left as is. MASS_CLAMP floors the mass only; PSD_PROJECT is the Frobenius-norm
    * projection of the pseudo-inertia onto the positive semidefinite cone (eigenvalue clipping), which makes the
    * estimate fully physically consistent without a solver.
    */
   /** LOGDET_MAP: the recursion runs on the unconstrained mean, so put it back before the prediction and residual. */
   @Override
   protected void preSolveHook()
   {
      if (stateConstraint == StateConstraint.LOGDET_MAP && unconstrainedMean != null)
         state.set(unconstrainedMean);
   }

   @Override
   public void postSolveHook()
   {
      if (stateConstraint == StateConstraint.LOGDET_MAP)
         applyLogDetMap(); // also while held: the mean is then unchanged, and the MAP of an unchanged mean is unchanged
      else if (stateConstraint != StateConstraint.NONE && !isParametersHeld())
         applyStateConstraint();

      for (int i = 0; i < piWatchers.size(); i++)
      {
         CommonOps_DDRM.extract(state, piWatcherOffsets.get(i), piWatcherOffsets.get(i) + PI_SIZE, 0, 1, piBlock, 0, 0);
         piWatchers.get(i).set(piBlock);
      }
   }

   private void applyLogDetMap()
   {
      if (unconstrainedMean == null)
      {
         unconstrainedMean = new DMatrixRMaj(state.getNumRows(), 1);
         logDetMapLast = new DMatrixRMaj(initialState);
      }
      unconstrainedMean.set(state);
      for (int offset = 0; offset + PI_SIZE <= state.getNumRows(); offset += PI_SIZE)
      {
         CommonOps_DDRM.extract(unconstrainedMean, offset, offset + PI_SIZE, 0, 1, piBlock, 0, 0);
         // W = (P_block)^-1 (cross-body coupling ignored; one body in this study)
         CommonOps_DDRM.extract(covariance, offset, offset + PI_SIZE, offset, offset + PI_SIZE, lmiW, 0, 0);
         for (int i = 0; i < PI_SIZE; i++)
            lmiW.add(i, i, 1.0e-12);
         DMatrixRMaj pBlock = lmiW.copy();
         if (!CommonOps_DDRM.invertSPD(pBlock, lmiW))
         {
            lmiW.set(pBlock);
            CommonOps_DDRM.invert(lmiW);
         }
         // prior body J0 = the nominal (initial) body
         CommonOps_DDRM.extract(initialState, offset, offset + PI_SIZE, 0, 1, lmiStart, 0, 0);
         PseudoInertiaConstrainedProjection.pseudoInertia(lmiStart, pseudoInertia);
         logDetJ0Inverse.set(pseudoInertia);
         CommonOps_DDRM.invert(logDetJ0Inverse);
         double logDetJ0 = logDetMapSolver.logDet(lmiStart);
         // warm start at the last MAP if strictly feasible, else the nominal body
         CommonOps_DDRM.extract(logDetMapLast, offset, offset + PI_SIZE, 0, 1, lmiResult, 0, 0);
         if (logDetMapSolver.isStrictlyConsistent(lmiResult))
            lmiStart.set(lmiResult);
         DMatrixRMaj prior = new DMatrixRMaj(PI_SIZE, 1);
         CommonOps_DDRM.extract(initialState, offset, offset + PI_SIZE, 0, 1, prior, 0, 0);
         if (!logDetMapSolver.logDetMap(piBlock, lmiW, prior, logDetMapWeight, lmiStart, lmiResult))
            lmiResult.set(lmiStart);
         for (int r = 0; r < PI_SIZE; r++)
         {
            state.set(offset + r, 0, lmiResult.get(r));
            logDetMapLast.set(offset + r, 0, lmiResult.get(r));
         }
         logDetMapDivergence.set(logDetMapSolver.logDetDivergence(lmiResult, logDetJ0Inverse, logDetJ0));
         logDetMapNewtonSteps.set(logDetMapSolver.getLastNewtonSteps());
         logDetMapDistance.set(logDetMapSolver.getLastDistance());
         logDetMapConverged.set(logDetMapSolver.wasLastConverged());
      }
   }

   private void applyStateConstraint()
   {

      for (int offset = 0; offset + PI_SIZE <= state.getNumRows(); offset += PI_SIZE)
      {
         if (stateConstraint == StateConstraint.MASS_CLAMP)
         {
            if (state.get(offset, 0) < MASS_FLOOR)
               state.set(offset, 0, MASS_FLOOR);
         }
         else if (stateConstraint == StateConstraint.LMI_MAP)
         {
            projectLmiMap(offset);
         }
         else
         {
            projectPseudoInertia(offset);
         }
      }
   }

   private void projectLmiMap(int offset)
   {
      if (lmiLast == null)
      {
         lmiLast = new DMatrixRMaj(state.getNumRows(), 1);
         lmiLast.set(initialState);
      }
      CommonOps_DDRM.extract(state, offset, offset + PI_SIZE, 0, 1, piBlock, 0, 0);
      // W = (P_block)^-1, the information of this body's block (cross-body coupling ignored; one body in this study)
      CommonOps_DDRM.extract(covariance, offset, offset + PI_SIZE, offset, offset + PI_SIZE, lmiW, 0, 0);
      for (int i = 0; i < PI_SIZE; i++)
         lmiW.add(i, i, 1.0e-12);
      DMatrixRMaj pBlock = lmiW.copy();
      if (!CommonOps_DDRM.invertSPD(pBlock, lmiW))
      {
         lmiW.set(pBlock);
         CommonOps_DDRM.invert(lmiW);
      }
      // warm start at the last projection if still strictly feasible, else the URDF body
      CommonOps_DDRM.extract(lmiLast, offset, offset + PI_SIZE, 0, 1, lmiStart, 0, 0);
      if (!lmiProjection.isStrictlyConsistent(lmiStart))
         CommonOps_DDRM.extract(initialState, offset, offset + PI_SIZE, 0, 1, lmiStart, 0, 0);
      if (!lmiProjection.project(piBlock, lmiW, lmiStart, lmiResult))
      {
         projectPseudoInertia(offset);
         return;
      }
      for (int r = 0; r < PI_SIZE; r++)
      {
         state.set(offset + r, 0, lmiResult.get(r));
         lmiLast.set(offset + r, 0, lmiResult.get(r));
      }
      lmiDistance.set(lmiProjection.getLastDistance());
      lmiNewtonSteps.set(lmiProjection.getLastNewtonSteps());
      lmiActive.set(lmiProjection.wasLastActive());
   }

   private void projectPseudoInertia(int offset)
   {
      double m = state.get(offset, 0);
      double hx = state.get(offset + 1, 0), hy = state.get(offset + 2, 0), hz = state.get(offset + 3, 0);
      double ixx = state.get(offset + 4, 0), ixy = state.get(offset + 5, 0), ixz = state.get(offset + 6, 0);
      double iyy = state.get(offset + 7, 0), iyz = state.get(offset + 8, 0), izz = state.get(offset + 9, 0);

      // J = [S h; h^T m] with S = 0.5 tr(I) I_3 - I
      double halfTrace = 0.5 * (ixx + iyy + izz);
      pseudoInertia.set(0, 0, halfTrace - ixx);
      pseudoInertia.set(1, 1, halfTrace - iyy);
      pseudoInertia.set(2, 2, halfTrace - izz);
      pseudoInertia.set(0, 1, -ixy);
      pseudoInertia.set(1, 0, -ixy);
      pseudoInertia.set(0, 2, -ixz);
      pseudoInertia.set(2, 0, -ixz);
      pseudoInertia.set(1, 2, -iyz);
      pseudoInertia.set(2, 1, -iyz);
      pseudoInertia.set(0, 3, hx);
      pseudoInertia.set(3, 0, hx);
      pseudoInertia.set(1, 3, hy);
      pseudoInertia.set(3, 1, hy);
      pseudoInertia.set(2, 3, hz);
      pseudoInertia.set(3, 2, hz);
      pseudoInertia.set(3, 3, m);

      if (!eigenDecomposition.decompose(pseudoInertia))
         return;
      boolean clipped = false;
      for (int i = 0; i < 4; i++)
         clipped |= eigenDecomposition.getEigenvalue(i).getReal() < EIGENVALUE_FLOOR;
      if (!clipped)
         return;

      // J+ = sum_i max(lambda_i, eps) v_i v_i^T
      projectedPseudoInertia.zero();
      for (int i = 0; i < 4; i++)
      {
         double lambda = Math.max(eigenDecomposition.getEigenvalue(i).getReal(), EIGENVALUE_FLOOR);
         DMatrixRMaj v = eigenDecomposition.getEigenVector(i);
         for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
               projectedPseudoInertia.add(r, c, lambda * v.get(r, 0) * v.get(c, 0));
      }

      // Back to pi: m = J33, h = J(0:2,3), I = tr(S) I_3 - S
      DMatrixRMaj J = projectedPseudoInertia;
      double traceS = J.get(0, 0) + J.get(1, 1) + J.get(2, 2);
      state.set(offset, 0, J.get(3, 3));
      state.set(offset + 1, 0, J.get(0, 3));
      state.set(offset + 2, 0, J.get(1, 3));
      state.set(offset + 3, 0, J.get(2, 3));
      state.set(offset + 4, 0, traceS - J.get(0, 0));
      state.set(offset + 5, 0, -J.get(0, 1));
      state.set(offset + 6, 0, -J.get(0, 2));
      state.set(offset + 7, 0, traceS - J.get(1, 1));
      state.set(offset + 8, 0, -J.get(1, 2));
      state.set(offset + 9, 0, traceS - J.get(2, 2));
   }

   @Override
   public void setRegressor(DMatrix regressorForEstimates)
   {
      this.regressor.set(regressorForEstimates);
   }

   @Override
   public void setTorqueFromNominal(DMatrix torqueFromNominal)
   {
      this.torqueFromNominal.set(torqueFromNominal);
   }

   @Override
   public void setTorqueFromBias(DMatrix bias)
   {
      this.torqueFromBias.set(bias);
   }

   @Override
   public void setContactJacobians(SideDependentList<DMatrixRMaj> jacobians)
   {
      for (RobotSide side : RobotSide.values)
         contactJacobians.get(side).set(jacobians.get(side));
   }

   @Override
   public void setContactWrenches(SideDependentList<DMatrixRMaj> contactWrenches)
   {
      for (RobotSide side : RobotSide.values)
         this.contactWrenches.get(side).set(contactWrenches.get(side));
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
}
