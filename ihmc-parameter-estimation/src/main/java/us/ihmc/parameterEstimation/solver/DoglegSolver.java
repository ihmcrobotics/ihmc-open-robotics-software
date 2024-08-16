package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.MatrixMissingTools;

/**
 * A garbage-free implementation of Powell's Dogleg method for solving nonlinear least squares problems.
 * <p>
 * This solver follows the implementation in Madsen, Nielsen, and Tingleff (2004): "Methods for Non-Linear Least Squares Problems" (pp. 29-32). This document is
 * linked in the following internal confluence page: <a href="https://confluence.ihmc.us/display/OSS/Online+Parameter+Estimation">LINK</a>
 * </p>
 * <p>
 * We consider nonlinear least squares problems of the form: 0.5 * || r(x) ||<sup>2</sup>
 * where r = r(x) is the residual of the problem. The Jacobian of the residual evaluated at x is denoted J(x).
 * </p>
 * <p>
 * The constructor of this solver takes the size of the problem variable and the size of the residual error output. From there, it will size all internal
 * variables appropriately.
 * </p>
 *
 * @author James Foster
 */
public class DoglegSolver
{
   private static final double EPSILON = 1.0e-12;

   private final ResidualAndJacobian residualAndJacobian;

   private double objective = 0.0;
   private final DMatrixRMaj residual;
   private final DMatrixRMaj jacobian;
   private final DMatrixRMaj gradient;
   private final DMatrixRMaj jacobianGradientProduct;

   private final DMatrixRMaj steepestDescentStep;
   private final DMatrixRMaj gaussNewtonStep;
   private final DMatrixRMaj doglegStep;

   private final LinearSolver<DMatrixRMaj, DMatrixRMaj> gaussNewtonStepSolver;
   private double gaussNewtonStepSystemQuality = 0.0;
   private double gaussNewtonStepSystemQualityTolerance = 1.0e-12;

   private final DMatrixRMaj residualCopy;
   private final DMatrixRMaj gradientCopy;

   private final DMatrixRMaj alphaModifiedSteepestDescentStep;
   private final DMatrixRMaj gnStepMinusAlphaSDStep;

   private double alpha = 0.0;
   private double beta = 0.0;

   private double trustRegionRadius = 0.0;

   private double linearizedObjectiveDecreasePrediction = 0.0;
   private double objectiveDecreaseRatioToLinearizedModel = 0.0;

   private int iteration = 0;
   private int maximumIterations;
   private boolean verbose = false;

   private final DMatrixRMaj x;
   private final DMatrixRMaj xNew;

   private double gradientConvergenceTolerance = 1.0e-9;
   private double radiusConvergenceTolerance = 1.0e-9;
   private double residualConvergenceTolerance = 1.0e-12;
   private boolean isConvergedViaGradient = false;
   private boolean isConvergedViaRadius = false;
   private boolean isConvergedViaResidual = false;
   private boolean isConverged = false;

   private DoglegStepType doglegStepType = DoglegStepType.NONE;
   private enum DoglegStepType
   {
      GAUSS_NEWTON, SCALED_STEEPEST_DESCENT, DOGLEG, NONE
   }

   public DoglegSolver(ResidualAndJacobian residualAndJacobian, int maximumIterations)
   {
      this.maximumIterations = maximumIterations;
      this.residualAndJacobian = residualAndJacobian;
      int parameterSize = residualAndJacobian.getParameterSize();
      int residualSize = residualAndJacobian.getResidualSize();

      residual = new DMatrixRMaj(residualSize, 1);
      residual.zero();

      jacobian = new DMatrixRMaj(residualSize, parameterSize);
      jacobian.zero();

      gradient = new DMatrixRMaj(parameterSize, 1);
      gradient.zero();

      jacobianGradientProduct = new DMatrixRMaj(residualSize, 1);
      jacobianGradientProduct.zero();

      steepestDescentStep = new DMatrixRMaj(parameterSize, 1);
      steepestDescentStep.zero();
      gaussNewtonStep = new DMatrixRMaj(parameterSize, 1);
      gaussNewtonStep.zero();
      // According to the EJML linear solver documentation, this should never fail, even for singular systems, but be more performant than an SVD solve. Some
      // accuracy may be lost as a compromise
      gaussNewtonStepSolver = LinearSolverFactory_DDRM.pseudoInverse(false);
      residualCopy = new DMatrixRMaj(residualSize, 1);
      residualCopy.zero();
      gradientCopy = new DMatrixRMaj(parameterSize, 1);
      gradientCopy.zero();

      doglegStep = new DMatrixRMaj(parameterSize, 1);
      doglegStep.zero();
      alphaModifiedSteepestDescentStep = new DMatrixRMaj(parameterSize, 1);
      alphaModifiedSteepestDescentStep.zero();
      gnStepMinusAlphaSDStep = new DMatrixRMaj(parameterSize, 1);
      gnStepMinusAlphaSDStep.zero();

      x = new DMatrixRMaj(parameterSize, 1);
      x.zero();
      xNew = new DMatrixRMaj(parameterSize, 1);
      xNew.zero();
   }

   /**
    * Reset solver by setting all variables back to zero. Will require re-initializing with {@link #initialize}.
    */
   public void reset()
   {
      objective = 0.0;
      residual.zero();
      jacobian.zero();
      gradient.zero();
      jacobianGradientProduct.zero();
      steepestDescentStep.zero();
      gaussNewtonStep.zero();
      doglegStep.zero();
      // gaussNewtonStepSolver -- no method to zero
      gaussNewtonStepSystemQuality = 0.0;
      // gaussNewtonStepSystemQualityTolerance
      residualCopy.zero();
      gradientCopy.zero();
      alphaModifiedSteepestDescentStep.zero();
      gnStepMinusAlphaSDStep.zero();
      alpha = 0.0;
      beta = 0.0;
      trustRegionRadius = 0.0;
      linearizedObjectiveDecreasePrediction = 0.0;
      objectiveDecreaseRatioToLinearizedModel = 0.0;
      iteration = 0;
      // maximumIterations -- we don't want to reset this, assume the user's preference is unchanged, they can use the setter
      x.zero();
      xNew.zero();
      // gradientConvergenceTolerance
      // radiusConvergenceTolerance
      // residualConvergenceTolerance
      isConvergedViaGradient = false;
      isConvergedViaRadius = false;
      isConvergedViaResidual = false;
      isConverged = false;
      doglegStepType = DoglegStepType.NONE;
   }

   /**
    * Calculate the residual error evaluated at problem variable x.
    *
    * @param x the problem variable to evaluate the residual error at.
    * @param residualToPack the vector in which to pack the residual error result.
    */
   private void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack)
   {
      residualAndJacobian.calculateResidual(x, residualToPack);
   }

   /**
    * Calculate the objective function value of the nonlinear least squares problem, given by:
    * <pre>
    * 0.5 * || r ||<sup>2</sup>
    * </pre>
    * where r = r(x) is the residual error function.
    *
    * @param residual the residual error used to calculate the objective function value.
    * @return the scalar objective function value corresponding to {@code residual}.
    */
   private double calculateObjective(DMatrixRMaj residual)
   {
      return 0.5 * CommonOps_DDRM.dot(residual, residual);
   }

   /**
    * Calculate the Jacobian of the residual error evaluated at problem variable x.
    *
    * @param x the problem variable to evaluate the Jacobian at.
    * @param jacobianToPack the matrix in which to pack the Jacobian result.
    */
   private void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack)
   {
      residualAndJacobian.calculateJacobian(x, jacobianToPack);
   }

   /**
    * Calculate the gradient of the objective function evaluated at problem variable x.
    * <p>
    * In nonlinear least squares the problems, the gradient has a special structure, and can be calculated as the Jacobian transpose multiplied by the residual
    * error. Be careful to ensure that the Jacobian and residual error are evaluated at the same problem variable x value!
    * </p>
    * @param jacobian the Jacobian to use to calculate the gradient.
    * @param residual the residual error to use to calculate the gradient.
    * @param gradientToPack the vector in which to pack the gradient result.
    */
   private void calculateGradient(DMatrixRMaj jacobian, DMatrixRMaj residual, DMatrixRMaj gradientToPack)
   {
      CommonOps_DDRM.multTransA(jacobian, residual, gradientToPack);
   }

   private void updateSteepestDescentStep()
   {
      steepestDescentStep.set(gradient);
      MatrixMissingTools.negate(steepestDescentStep);
   }

   private void updateGaussNewtonStep()
   {
      residualCopy.set(residual);
      gaussNewtonStepSolver.setA(jacobian);
      // Check "quality" of system to solve -- results close to or equal to zero indicate a near-singular or singular system
      gaussNewtonStepSystemQuality = gaussNewtonStepSolver.quality();
      MatrixMissingTools.negate(residualCopy);
      gaussNewtonStepSolver.solve(residualCopy, gaussNewtonStep);
   }

   private void updateDoglegStep()
   {
      CommonOps_DDRM.scale(alpha, steepestDescentStep, alphaModifiedSteepestDescentStep);
      // If Gauss-Newton step is not trivial due to Jacobian rank-deficiency, and it is within trust region, use it
      if (gaussNewtonStepSystemQuality > gaussNewtonStepSystemQualityTolerance && NormOps_DDRM.fastNormP2(gaussNewtonStep) < trustRegionRadius)
      {
         doglegStep.set(gaussNewtonStep);
         doglegStepType = DoglegStepType.GAUSS_NEWTON;
      }

      // If alpha * steepestDescentStep is outside trust region, scale it to touch the edge of the trust region and use it
      else if (NormOps_DDRM.fastNormP2(alphaModifiedSteepestDescentStep) >= trustRegionRadius)
      {
         CommonOps_DDRM.scale(trustRegionRadius / NormOps_DDRM.fastNormP2(steepestDescentStep), steepestDescentStep, doglegStep);
         doglegStepType = DoglegStepType.SCALED_STEEPEST_DESCENT;
      }
      // Otherwise, with alpha already calculated, calculate beta such that alpha * steepestDescentStep + beta * (gaussNewtonStep - alpha * steepestDescentStep)
      // touches the edge of the trust region
      else
      {
         updateBeta();
         doglegStep.set(alphaModifiedSteepestDescentStep);
         CommonOps_DDRM.addEquals(doglegStep, beta, gaussNewtonStep);
         CommonOps_DDRM.addEquals(doglegStep, -beta, alphaModifiedSteepestDescentStep);
         doglegStepType = DoglegStepType.DOGLEG;
      }
   }

   private void updateAlpha()
   {
      CommonOps_DDRM.mult(jacobian, gradient, jacobianGradientProduct);
      alpha = CommonOps_DDRM.dot(gradient, gradient) / CommonOps_DDRM.dot(jacobianGradientProduct, jacobianGradientProduct);
   }

   private void updateBeta()
   {
      // Update container variable (and recreate cheap double)
      CommonOps_DDRM.subtract(gaussNewtonStep, alphaModifiedSteepestDescentStep, gnStepMinusAlphaSDStep);
      double c = CommonOps_DDRM.dot(alphaModifiedSteepestDescentStep, gnStepMinusAlphaSDStep);
      // Calculating common expressions to both conditionals
      double exp1 = MathTools.square(trustRegionRadius) - CommonOps_DDRM.dot(alphaModifiedSteepestDescentStep, alphaModifiedSteepestDescentStep);
      double exp2 = CommonOps_DDRM.dot(gnStepMinusAlphaSDStep, gnStepMinusAlphaSDStep);
      double exp3 = Math.sqrt(MathTools.square(c) + exp2 * exp1);
      if (c <= 0)
         beta = (-c + exp3) / exp2;
      else
         beta = exp1 / (c + exp3);
   }

   private boolean isGradientStoppingCriteriaMet()
   {
      return NormOps_DDRM.fastNormP(gradient, Double.POSITIVE_INFINITY) < gradientConvergenceTolerance;
   }

   private boolean isRadiusStoppingCriteriaMet(double toCompare)
   {
      return toCompare < radiusConvergenceTolerance * (NormOps_DDRM.fastNormP2(x) + radiusConvergenceTolerance);
   }

   private boolean isResidualStoppingCriteriaMet()
   {
      return NormOps_DDRM.fastNormP(residual, Double.POSITIVE_INFINITY) < residualConvergenceTolerance;
   }

   /**
    * Initialize the solver with an {@code initialGuess} of {@code x} and a starting {@code trustRegionRadius}.
    *
    * @param initialGuess the starting valye of {@code x} from which to start the solver.
    * @param trustRegionRadius the initial trust region radius to start the solver with.
    */
   public void initialize(DMatrixRMaj initialGuess, double trustRegionRadius)
   {
      iteration = 0;
      x.set(initialGuess);
      this.trustRegionRadius = trustRegionRadius;
   }

   /**
    * Solve the nonlinear least squares problem.
    * <p>
    * NOTE: One must call {@link #initialize} before calling this method in order to correctly set up the optimization problem. For more details on the
    * mathematics of this particular Powell's Dogleg implementation, see the documents linked in the class documentation.
    * </p>
    */
   public void solve()
   {
      while (!isConverged && iteration < maximumIterations)
      {
         iteration += 1;

         // Each iteration, update the major problem objects (and optionally print them)
         updateProblemObjects();
         if (verbose)
            LogTools.info("Objective: " + objective + " | " + "Gradient Norm: " + NormOps_DDRM.fastNormP2(gradient) + " | " + "Trust Region Radius: " + trustRegionRadius + " | " + "Step Type: " + doglegStepType.name());

         // The updated problem objects allow us to update the descent directions
         updateProblemSteps();

         // Check if the Dogleg step is sufficiently small to merit convergence
         if (isRadiusStoppingCriteriaMet(NormOps_DDRM.fastNormP2(doglegStep)))
         {
            isConvergedViaRadius = true;
            isConverged = true;
         }
         // Otherwise, update the problem variable x with the Dogleg step, and calculate the ratio of the objective function decrease compared to that predicted
         // by a linearized model of the objective
         else
         {
            CommonOps_DDRM.add(x, doglegStep, xNew);
            calculateObjectiveDecreaseRatioToLinearizedModel();

            // The ratio being positive indicates that we are descending. Therefore, we accept xNew as an update to x, update, and move on
            if (objectiveDecreaseRatioToLinearizedModel > 0)
            {
               x.set(xNew);
               updateProblemObjects();
               isConvergedViaResidual = isResidualStoppingCriteriaMet();
               isConvergedViaGradient = isGradientStoppingCriteriaMet();
            }
            // Furthermore, if the ratio is quite high, we can expand the trust region to hopefully take larger and larger Gauss-Newton steps
            if (objectiveDecreaseRatioToLinearizedModel > 0.75)
               trustRegionRadius = Math.max(trustRegionRadius, 3 * NormOps_DDRM.fastNormP2(doglegStep));
            // But if the ratio is only modest to low, we shrink the trust region to maintain validity
            else if (objectiveDecreaseRatioToLinearizedModel < 0.25)
            {
               trustRegionRadius = trustRegionRadius / 2;
               isConvergedViaRadius = isRadiusStoppingCriteriaMet(trustRegionRadius);
            }
            isConverged = isConvergedViaResidual || isConvergedViaGradient || isConvergedViaRadius;
         }
      }
      if (verbose)
         logSolverResults();
   }

   /**
    * Update the major problem objects: {@link #residual}, {@link #objective}, {@link #jacobian}, and {@link #gradient}, at a new value of the problem
    * variable {@link #x}.
    * <p>
    * NOTE: This is an internal method of {@link #solve}, and it is not recommended to call this method outside of testing, or unless you know what you're
    * doing. Refer to {@link #solve} for which methods should be called before / after this one.
    * </p>
    */
   void updateProblemObjects()
   {
      calculateResidual(x, residual);
      objective = calculateObjective(residual);
      calculateJacobian(x, jacobian);
      calculateGradient(jacobian, residual, gradient);
   }

   /**
    * Update the descent steps used in the problem: {@link #steepestDescentStep}, {@link #gaussNewtonStep}, {@link #doglegStep}.
    * <p>
    * NOTE: This is an internal method of {@link #solve}, and it is not recommended to call this method outside of testing, or unless you know what you're
    * doing. Refer to {@link #solve} for which methods should be called before / after this one.
    * </p>
    */
   void updateProblemSteps()
   {
      updateAlpha();
      // Now compute the steepest descent and Gauss-Newton steps for forming the Dogleg
      updateSteepestDescentStep();
      updateGaussNewtonStep();
      // Now with the two component steps, we can form the Dogleg step
      updateDoglegStep();
   }

   /**
    * Calculate the ratio of the {@link #objective} decrease between {@link #x} and {@link #xNew} compared to that predicted by a linearized model of the
    * objective function.
    * <p>
    * NOTE: This is an internal method of {@link #solve}, and it is not recommended to call this method outside of testing, or unless you know what you're
    * doing. Refer to {@link #solve} for which methods should be called before / after this one.
    * </p>
    */
   private void calculateObjectiveDecreaseRatioToLinearizedModel()
   {
      // Get objective function values for x and xNew
      double objectiveAtX = objective;
      calculateResidual(xNew, residualCopy); // we're going to use the residualCopy to temporarily store the residual for xNew
      double objectiveAtXNew = calculateObjective(residualCopy);

      // In the following conditionals, one of the checks compares the Dogleg step to a scaled multiple of the gradient, which we construct beforehand here
      gradientCopy.set(gradient);
      CommonOps_DDRM.scale(-trustRegionRadius / NormOps_DDRM.fastNormP2(gradient), gradientCopy);

      // If the Dogleg step is just the Gauss-Newton step...
      if (MatrixFeatures_DDRM.isEquals(doglegStep, gaussNewtonStep, EPSILON))
         linearizedObjectiveDecreasePrediction = objectiveAtX;
         // Else if the Dogleg step is the steepest descent step that touches the edge of the trust region...
      else if (MatrixFeatures_DDRM.isEquals(doglegStep, gradientCopy, EPSILON))
      {
         gradientCopy.set(gradient);
         CommonOps_DDRM.scale(alpha, gradientCopy);
         linearizedObjectiveDecreasePrediction = (trustRegionRadius * (2 * NormOps_DDRM.fastNormP2(gradientCopy) - trustRegionRadius)) / 2 * alpha;
      }
      // Otherwise...
      else
         linearizedObjectiveDecreasePrediction = 0.5 * alpha * MathTools.square(1 - beta) * CommonOps_DDRM.dot(gradient, gradient) + beta * (2 - beta) * objective;

      objectiveDecreaseRatioToLinearizedModel = (objectiveAtX - objectiveAtXNew) / linearizedObjectiveDecreasePrediction;
   }

   private void logSolverResults()
   {
      if (isConverged)
      {
         LogTools.info("CONVERGED! in " + iteration + " iterations.");
         if (isConvergedViaGradient)
            LogTools.info("Gradient norm under inf-norm convergence tolerance.");
         if (isConvergedViaRadius)
            LogTools.info("Dogleg / trust region step under 2-norm convergence tolerance.");
         if (isConvergedViaResidual)
            LogTools.info("Residual norm under inf-norm convergence tolerance.");
         LogTools.info("Solution: " + x.toString());
      }
      else
         LogTools.info("FAILED.");
   }

   public DMatrixRMaj getResidual()
   {
      return residual;
   }

   public double getObjective()
   {
      return objective;
   }

   public DMatrixRMaj getJacobian()
   {
      return jacobian;
   }

   public DMatrixRMaj getGradient()
   {
      return gradient;
   }

   public double getAlpha()
   {
      return alpha;
   }

   public double getBeta()
   {
      return beta;
   }

   public DMatrixRMaj getSteepestDescentStep()
   {
      return steepestDescentStep;
   }

   public DMatrixRMaj getGaussNewtonStep()
   {
      return gaussNewtonStep;
   }

   public DMatrixRMaj getDoglegStep()
   {
      return doglegStep;
   }

   public DMatrixRMaj getX()
   {
      return x;
   }

   public boolean isConverged()
   {
      return isConverged;
   }

   public double getGradientConvergenceTolerance()
   {
      return gradientConvergenceTolerance;
   }

   public double getRadiusConvergenceTolerance()
   {
      return radiusConvergenceTolerance;
   }

   public double getResidualConvergenceTolerance()
   {
      return residualConvergenceTolerance;
   }

   public double getGaussNewtonStepSystemQualityTolerance()
   {
      return gaussNewtonStepSystemQualityTolerance;
   }

   public void setGradientConvergenceTolerance(double gradientConvergenceTolerance)
   {
      this.gradientConvergenceTolerance = gradientConvergenceTolerance;
   }

   public void setRadiusConvergenceTolerance(double radiusConvergenceTolerance)
   {
      this.radiusConvergenceTolerance = radiusConvergenceTolerance;
   }

   public void setResidualConvergenceTolerance(double residualConvergenceTolerance)
   {
      this.residualConvergenceTolerance = residualConvergenceTolerance;
   }

   public void setGaussNewtonStepSystemQualityTolerance(double gaussNewtonStepSystemQualityTolerance)
   {
      this.gaussNewtonStepSystemQualityTolerance = gaussNewtonStepSystemQualityTolerance;
   }

   public void setMaximumIterations(int maximumIterations)
   {
      this.maximumIterations = maximumIterations;
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }
}
