package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;

import java.util.ArrayList;
import java.util.List;

/**
 * Converts the constrained optimization problem:
 *    minimize f(x)
 *    st: Gi(x) == 0 for G[]
 *        Hi(x) >= 0 for H[]
 * to an iterative unconstrained problem:
 *    minimize F(x) = f(x) + <l, G(x)> + p/2 <G(x), G(x)> + (stuff for H(x))
 * using the Augmented Lagrangian method:
 *    Lagrangian: f(x) + <l, G(x)>
 *    Penalty: f(x) + p/2 <G(x), G(x)>
 * The actual optimizer is implemented separately.
 *
 * An example use case is:
 *    ALOP lagrangian = new ALOP(costFunction, inequalityConstraints, equalityConstraints)
 *    CostFunction augmentedCostFunction = lagrangian.calculateDualProblemCost
 *
 *    Optimizer optimizer = new Optimizer(augmentedCostFunction)
 *    for (n lagrangian steps)
 *       optimum = optimizer.doRun()
 *       lagrangian.updateLagrangeMultipliers(optimum)
 */
public class AugmentedLagrangeOptimizationProblem
{
   private CostFunction costFunction;
   private List<ConstraintFunction> inequalityConstraints = new ArrayList<>();
   private List<ConstraintFunction> equalityConstraints = new ArrayList<>();

   DMatrixD1 equalityMultiplier = new DMatrixRMaj(new double[] {}); // TODO starting value
   DMatrixD1 inequalityMultiplier = new DMatrixRMaj(new double[] {});
   double penalty;
   private double penaltyIncreaseFactor;

   public AugmentedLagrangeOptimizationProblem(CostFunction costFunction)
   {
      this.costFunction = costFunction;
   }

   public void addInequalityConstraint(ConstraintFunction constraint)
   {
      inequalityConstraints.add(constraint);
   }

   public void addEqualityConstraint(ConstraintFunction constraint)
   {
      equalityConstraints.add(constraint);
   }

   public void initialize(double initialPenalty, double penaltyIncreaseFactor)
   {
      equalityMultiplier = new DMatrixRMaj(equalityConstraints.size(), 1);
      inequalityMultiplier = new DMatrixRMaj(inequalityConstraints.size(), 1);

      this.penalty = initialPenalty;
      this.penaltyIncreaseFactor = penaltyIncreaseFactor;
   }

   public double calculateDualProblemCost(DMatrixD1 x)
   {
      return calculateAugmentedLagrangian(equalityMultiplier, inequalityMultiplier, penalty, x);
   }

   private double calculateAugmentedLagrangian(DMatrixD1 equalityMultiplierK, DMatrixD1 inequalityMultiplierK, double penalty, DMatrixD1 x)
   {
      double objectivesCost = calculateObjectives(x);

      // Inequalities Cost =
      //    if H(x) >= multiplier / p (not violated barrier): -1/(2p) <multiplier, multiplier>
      //    if H(x) <  multiplier / p (violated barrier)    : -<multiplier, H(x)> + p/2 ||H(x)||^2
      // The below formula ends up being equivalent to above.
      DMatrixD1 inequalityConstraintBarrierValue = calculateBarrierInequalityConstraintValue(x);
      double inequalityConstraintCost = 1 / (2.0 * penalty) * (
                                          VectorVectorMult_DDRM.innerProd(inequalityConstraintBarrierValue, inequalityConstraintBarrierValue) -
                                          VectorVectorMult_DDRM.innerProd(inequalityMultiplierK, inequalityMultiplierK)
                                       );

      // Equalities Cost = p/2 * ||C(x)||^2 + <multiplier, C(x)>
      DMatrixD1 equalityConstraintValue = calculateEqualityConstraintValue(x);
      double equalityConstraintCost = penalty / 2.0 * VectorVectorMult_DDRM.innerProd(equalityConstraintValue, equalityConstraintValue) +
                                      VectorVectorMult_DDRM.innerProd(equalityMultiplierK, equalityConstraintValue);

      return objectivesCost + inequalityConstraintCost + equalityConstraintCost;
   }

   public void updateLagrangeMultipliers(DMatrixD1 xOptimal)
   {
      // Inequalities Cost =
      //    if H(x') >= multiplier / p (not violated barrier): multiplier = 0
      //    if H(x') <  multiplier / p (violated barrier)    : multiplier += p * H(x')
      inequalityMultiplier.set(calculateBarrierInequalityConstraintValue(xOptimal));

      // multiplier += p * G(x')
      DMatrixD1 copyEqualityMultiplier = new DMatrixRMaj(equalityMultiplier);
      CommonOps_DDRM.add(copyEqualityMultiplier, penalty, calculateEqualityConstraintValue(xOptimal), equalityMultiplier);

      // increase penalty
      penalty *= penaltyIncreaseFactor;
   }

   private double calculateObjectives(DMatrixD1 x)
   {
      return costFunction.calculate(x);
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] >= 0, calculate [h1(x), h2(x), ...]
    */
   private DMatrixD1 calculateInequalityConstraintValue(DMatrixD1 x)
   {
      int numConstraints = inequalityConstraints.size();
      double[] value = new double[numConstraints];
      for (int i = 0; i < numConstraints; i++)
      {
         value[i] = inequalityConstraints.get(i).calculate(x);
      }
      return new DMatrixRMaj(value);
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] >= 0, calculate:
    *    [max(0, mult1 - p h1(x)), max(0, mult2 - p h2(x)), ...]
    *    aka max_elem(0_vec, multiplier - p H(x))
    */
   private DMatrixD1 calculateBarrierInequalityConstraintValue(DMatrixD1 x)
   {
      DMatrixD1 barrierInequalityValue = new DMatrixRMaj(inequalityMultiplier);
      CommonOps_DDRM.add(inequalityMultiplier, -penalty, calculateInequalityConstraintValue(x), barrierInequalityValue);

      // Perform the clamp
      int numConstraints = inequalityConstraints.size();
      for (int i = 0; i < numConstraints; i++)
      {
         if (barrierInequalityValue.get(i) < 0)
         {
            barrierInequalityValue.set(i, 0);
         }
      }
      return barrierInequalityValue;
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] = 0, calculate [h1(x), h2(x), ...]
    * @return
    */
   private DMatrixD1 calculateEqualityConstraintValue(DMatrixD1 x)
   {
      int numConstraints = equalityConstraints.size();
      double[] value = new double[numConstraints];
      for (int i = 0; i < numConstraints; i++)
      {
         value[i] = equalityConstraints.get(i).calculate(x);
      }
      return new DMatrixRMaj(value);
   }
}
