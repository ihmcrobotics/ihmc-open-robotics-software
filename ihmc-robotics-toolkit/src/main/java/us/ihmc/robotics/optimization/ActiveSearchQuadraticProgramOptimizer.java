package us.ihmc.robotics.optimization;

import java.util.Set;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;

import us.ihmc.commons.MathTools;

/**
 * @author twan
 *         Date: 8/9/13
 * based on Escande, Mansard, Wieber. Hierarchical Quadratic Programming - Part 1: Fundamental Bases. Algorithm 6.
 */
public class ActiveSearchQuadraticProgramOptimizer
{
   private QuadraticProgram quadraticProgram;
   private final ActiveSearchOptimizationSettings settings;
   private final ActiveSearchSolutionInfo solutionInfo = new ActiveSearchSolutionInfo();
   private final EqualityConstraintEnforcer equalityConstraintEnforcer;

   // temp stuff
   private final DMatrixRMaj cX = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj aCopy = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj bCopy = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj cActive = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj dActive = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj xBarStar = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj ci = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj step = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj axMinusB = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj lambda = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj aCPlus = new DMatrixRMaj(1, 1);

   public ActiveSearchQuadraticProgramOptimizer(ActiveSearchOptimizationSettings settings)
   {
      this.settings = settings;
      this.equalityConstraintEnforcer = new EqualityConstraintEnforcer(settings.getLinearSolver());
   }

   public void setQuadraticProgram(QuadraticProgram quadraticProgram)
   {
      this.quadraticProgram = quadraticProgram;
   }

   private void setInitialGuess(DMatrixRMaj initialGuess)
   {
      solutionInfo.setSolution(initialGuess);
      determineActiveSet(initialGuess);
   }

   private void determineActiveSet(DMatrixRMaj x)
   {
      solutionInfo.clearActiveSet();

      DMatrixRMaj c = quadraticProgram.getC();
      DMatrixRMaj d = quadraticProgram.getD();

      cX.reshape(quadraticProgram.getInequalityConstraintSize(), 1);
      CommonOps_DDRM.mult(c, x, cX);

      for (int i = 0; i < cX.getNumRows(); i++)
      {
         double cXi = cX.get(i, 0);
         double di = d.get(i, 0);

         if (MathTools.epsilonEquals(cXi, di, settings.getEpsilonConstraintActive()))
            solutionInfo.getActiveSet().add(i);
         else if (cXi >= di)
            throw new RuntimeException("x is not feasible!");
      }
   }

   public void solve(DMatrixRMaj initialGuess)
   {
      if (quadraticProgram == null)
         throw new RuntimeException("Quadratic program has not been set!");

      solutionInfo.reset(quadraticProgram.getSolutionSize());

      setInitialGuess(initialGuess);

      while (!solutionInfo.isConverged() && (solutionInfo.getIterations() < settings.getMaxIterations()))
      {
         solutionInfo.setConverged(true);

         updateActiveConstraintEquation();
         DMatrixRMaj xStar = computeOptimumForCurrentActiveSet();

         // compute step length
         DMatrixRMaj x = solutionInfo.getSolution();
         step.set(xStar);
         CommonOps_DDRM.subtractEquals(step, x); // step = xStar - x
         ci.reshape(1, quadraticProgram.getSolutionSize());

         double tau = 1.0;
         int newActiveConstraintIndex = -1;
         for (int constraintIndex = 0; constraintIndex < quadraticProgram.getInequalityConstraintSize(); constraintIndex++)
         {
            if (!solutionInfo.getActiveSet().contains(constraintIndex))
            {
               // c_i
               CommonOps_DDRM.extract(quadraticProgram.getC(), constraintIndex, constraintIndex + 1, 0, quadraticProgram.getSolutionSize(), ci, 0, 0);

               // d_i
               double di = quadraticProgram.getD().get(constraintIndex, 0);

               // ci * x
               double ciX = VectorVectorMult_DDRM.innerProd(ci, x);

               // ci * step
               double ciStep = VectorVectorMult_DDRM.innerProd(ci, step);

               // tau_i
               double tauI = -(ciX - di) / ciStep;

               // update tau
               if (tauI < tau)
               {
                  tau = tauI;
                  newActiveConstraintIndex = constraintIndex;
               }
            }
         }

         updateSolution(tau);

         // TODO: does order matter?
         decreaseActiveSetIfNecessary();
         increaseActiveSetIfNecessary(newActiveConstraintIndex);

         solutionInfo.incrementIterations();
      }
   }

   private void updateActiveConstraintEquation()
   {
      Set<Integer> activeSet = solutionInfo.getActiveSet();
      int nActiveConstraints = activeSet.size();

      DMatrixRMaj c = quadraticProgram.getC();
      DMatrixRMaj d = quadraticProgram.getD();

      cActive.reshape(nActiveConstraints, quadraticProgram.getSolutionSize());
      dActive.reshape(nActiveConstraints, 1);

      int activeConstraintIndex = 0;
      for (Integer constraintIndex : activeSet)
      {
         CommonOps_DDRM.extract(c, constraintIndex, constraintIndex + 1, 0, quadraticProgram.getSolutionSize(), cActive,
               activeConstraintIndex, 0);
         dActive.set(activeConstraintIndex, 0, d.get(constraintIndex, 0));

         activeConstraintIndex++;
      }
   }

   private DMatrixRMaj computeOptimumForCurrentActiveSet()
   {
      // compute the optimum for the current active set (treat all active constraints as equalities)
      aCopy.set(quadraticProgram.getA());
      bCopy.set(quadraticProgram.getB());

      equalityConstraintEnforcer.setConstraint(cActive, dActive);
      equalityConstraintEnforcer.constrainEquation(aCopy, bCopy);

      settings.getLinearSolver().setA(aCopy);
      xBarStar.reshape(quadraticProgram.getSolutionSize(), 1);
      settings.getLinearSolver().solve(bCopy, xBarStar);

      return equalityConstraintEnforcer.constrainResult(xBarStar);
   }

   private void increaseActiveSetIfNecessary(int newActiveConstraintIndex)
   {
      // if necessary, increase the active set
      if (newActiveConstraintIndex >= 0)
      {
         solutionInfo.getActiveSet().add(newActiveConstraintIndex);
         solutionInfo.setConverged(false);
      }
   }

   private void decreaseActiveSetIfNecessary()
   {
      // if necessary, decrease the active set
      // first compute lambda (solution to dual problem)
      axMinusB.reshape(quadraticProgram.getObjectiveSize(), 1);
      CommonOps_DDRM.mult(quadraticProgram.getA(), solutionInfo.getSolution(), axMinusB);
      CommonOps_DDRM.subtractEquals(axMinusB, quadraticProgram.getB());
      aCPlus.reshape(quadraticProgram.getObjectiveSize(), cActive.getNumRows());
      CommonOps_DDRM.mult(quadraticProgram.getA(), equalityConstraintEnforcer.getConstraintPseudoInverse(), aCPlus);
      lambda.reshape(cActive.getNumRows(), 1);
      CommonOps_DDRM.multTransA(aCPlus, axMinusB, lambda);
      CommonOps_DDRM.changeSign(lambda);
      // TODO: do you need to update cActive after possible changes in the active set due to increaseActiveSetIfNecessary?

      // compute nu
      double nu = Double.POSITIVE_INFINITY;
      Integer superfluousActiveConstraintIndex = null;
      int activeConstraintIndex = 0;
      for (Integer constraintIndex : solutionInfo.getActiveSet())
      {
         double lambdaI = lambda.get(activeConstraintIndex++, 0);
         if (lambdaI < nu)
         {
            nu = lambdaI;
            superfluousActiveConstraintIndex = constraintIndex;
         }
      }
      if (nu < 0.0)
      {
         solutionInfo.getActiveSet().remove(superfluousActiveConstraintIndex);
         solutionInfo.setConverged(false);
      }
   }

   private void updateSolution(double tau)
   {
      // update solution
      CommonOps_DDRM.scale(tau, step);
      CommonOps_DDRM.addEquals(solutionInfo.getSolution(), step);
   }

   public ActiveSearchSolutionInfo getSolutionInfo()
   {
      return solutionInfo;
   }
}
