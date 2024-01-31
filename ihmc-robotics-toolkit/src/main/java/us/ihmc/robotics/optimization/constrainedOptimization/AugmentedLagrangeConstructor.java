package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

/**
 * Reference: Numerical Optimization by Jorge Nocedal, Stephen J. Write
 *
 * Converts the constrained optimization problem:
 *    minimize f(x)
 *    st: Gi(x) == 0 for G[](x)
 *        Hi(x) >= 0 for H[](x)
 * to an iterative unconstrained problem:
 *    minimize F(x) = f(x) + <l, G(x)> + p/2 <G(x), G(x)> + (stuff for H(x))
 * using the Augmented Lagrangian method:
 *    Lagrangian: f(x) + <l, G(x)>
 *    Penalty: f(x) + p/2 <G(x), G(x)>
 *
 * This class performs the conversion of f(x), G[](x), and H[](x) -> F(x)
 * An augmented lagrange problem gets solved over multiple iterations of optimizations in which the lagrange
 * multipliers, l, are updated by the optima found in each iteration.
 */
public class AugmentedLagrangeConstructor
{
   private double penalty;
   private final double penaltyIncreaseFactor;

   private final DMatrixD1 equalityMultiplier;
   private final DMatrixD1 inequalityMultiplier;

   public AugmentedLagrangeConstructor(double initialPenalty, double penaltyIncreaseFactor, int numEqualityConstraints, int numInequalityConstraints)
   {
      equalityMultiplier = new DMatrixRMaj(numEqualityConstraints, 1);
      inequalityMultiplier = new DMatrixRMaj(numInequalityConstraints, 1);

      this.penalty = initialPenalty;
      this.penaltyIncreaseFactor = penaltyIncreaseFactor;
   }

   /**
    *  For the constrained optimization problem:
    *     minimize f(x)
    *     st: Gi(x) == 0 for G[]
    *         Hi(x) >= 0 for H[],
    *     return the augmented lagrange F(x)
    * @param originalCost is f(x)
    * @param inequalityConstraintEvaluations G[](x)
    * @param equalityConstraintEvaluations H[](x)
    * @return the augmented lagrangian cost
    */
   public double getAugmentedLagrangeCost(double originalCost, DMatrixD1 equalityConstraintEvaluations, DMatrixD1 inequalityConstraintEvaluations)
   {
      // Inequalities Cost =
      //    if H(x) >= multiplier / p (not violated barrier): -1/(2p) <multiplier, multiplier>
      //    if H(x) <  multiplier / p (violated barrier)    : -<multiplier, H(x)> + p/2 ||H(x)||^2
      // The below formula ends up being equivalent to above.
      double inequalityConstraintCost = getInequalityConstraintCost(inequalityConstraintEvaluations, inequalityMultiplier, penalty);

      // Equalities Cost = p/2 * ||C(x)||^2 + <multiplier, C(x)>
      double equalityConstraintCost = getEqualityConstraintCost(equalityConstraintEvaluations, equalityMultiplier, penalty);

      return originalCost + inequalityConstraintCost + equalityConstraintCost;
   }

   /**
    * @param equalityConstraintValue The value G(x) = [g1(x), g2(x), ...]
    * @param equalityMultiplierK Lagrange multiplier
    * @param penalty penalty term
    * @return
    */
   private static double getEqualityConstraintCost(DMatrixD1 equalityConstraintValue, DMatrixD1 equalityMultiplierK, double penalty)
   {
      return penalty / 2.0 * VectorVectorMult_DDRM.innerProd(equalityConstraintValue, equalityConstraintValue) +
             VectorVectorMult_DDRM.innerProd(equalityMultiplierK, equalityConstraintValue);
   }

   /**
    * @param inequalityConstraintValues The value H(x) = [h1(x), h2(x), ...]
    * @param inequalityMultiplier Lagrange multiplier
    * @param penalty penalty term
    * @return
    */
   private static double getInequalityConstraintCost(DMatrixD1 inequalityConstraintValues, DMatrixD1 inequalityMultiplier, double penalty)
   {
      DMatrixD1 inequalityConstraintBarrierValue = calculateBarrierInequalityConstraintValue(inequalityConstraintValues, inequalityMultiplier, penalty);
      return 1.0 / (2.0 * penalty) * (
            VectorVectorMult_DDRM.innerProd(inequalityConstraintBarrierValue, inequalityConstraintBarrierValue) -
            VectorVectorMult_DDRM.innerProd(inequalityMultiplier, inequalityMultiplier)
      );
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] >= 0, calculate:
    *    [max(0, mult1 - p h1(x)), max(0, mult2 - p h2(x)), ...]
    *    aka max_elem(0_vec, multiplier - p H(x))
    * @param inequalityConstraintValue The value H(x) = [h1(x), h2(x), ...]
    */
   private static DMatrixD1 calculateBarrierInequalityConstraintValue(DMatrixD1 inequalityConstraintValue,
                                                                     DMatrixD1 inequalityMultiplier,
                                                                     double penalty)
   {
      DMatrixD1 barrierInequalityValue = new DMatrixRMaj(inequalityMultiplier);
      CommonOps_DDRM.add(inequalityMultiplier, -penalty, inequalityConstraintValue, barrierInequalityValue);
      // Perform the clamp
      MatrixToolsLocal.elementWiseMin(barrierInequalityValue, 0.0);
      return barrierInequalityValue;
   }

   /**
    * For constraints
    *    H(x) = [h1(x), h2(x), ...] >= 0,
    *    G(x) = [g1(x), g2(x), ...] == 0
    * and the optimal results x* from minimizing F(x) in the last iteration,
    * updates the lagrange multiplier terms
    *
    * @param optimalEqualityConstraintEvaluations G(x*)
    * @param optimalInequalityConstraintEvaluations H(x*)
    */
   public void updateLagrangeMultipliers(DMatrixD1 optimalEqualityConstraintEvaluations, DMatrixD1 optimalInequalityConstraintEvaluations)
   {
      // Inequalities Cost =
      //    if H(x') >= multiplier / p (not violated barrier): multiplier = 0
      //    if H(x') <  multiplier / p (violated barrier)    : multiplier += p * H(x')
      inequalityMultiplier.set(calculateBarrierInequalityConstraintValue(optimalInequalityConstraintEvaluations, inequalityMultiplier, penalty));

      // multiplier += p * G(x')
      CommonOps_DDRM.addEquals(equalityMultiplier, penalty, optimalEqualityConstraintEvaluations);

      // increase penalty
      penalty *= penaltyIncreaseFactor;
   }
}
