package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.optimization.CostFunction;

import java.util.ArrayList;
import java.util.List;

/**
 * Reference: Numerical Optimization by Jorge Nocedal, Stephen J. Write
 * Converts the constrained optimization problem on parameters x:
 *    minimize f(x)
 *    st: Gi(x) == 0 for G[]
 *        Hi(x) >= 0 for H[]
 * to an iterative unconstrained problem:
 *    minimize F(x) = f(x) + <l, G(x)> + p/2 <G(x), G(x)> + (stuff for H(x))
 * using the Augmented Lagrangian method:
 *    Lagrangian: f(x) + <l, G(x)>
 *    Penalty: f(x) + p/2 <G(x), G(x)>
 *
 * This problem gets solved by {@link AugmentedLagrangeOptimizer}
 *
 * An example use case is:
 *    AugmentedLagrangeOptimizationProblem lagrangian = new AugmentedLagrangeOptimizationProblem(costFunction);
 *    lagrangian.addInequalityConstraint(inequalityConstraint);
 *    lagrangian.addEqualityConstraint(equalityConstraints);
 *    lagrangian.initialize(penalty, initialPenalty);
 *
 *    CostFunction augmentedCostFunction = lagrangian.calculateDualProblemCost
 *
 *    Optimizer optimizer = new Optimizer(augmentedCostFunction)
 *    for (n lagrangian steps)
 *       initialSeed = lastOptimum
 *       optimum = optimizer.doRun(initialSeed)
 *       lagrangian.updateLagrangeMultipliers(optimum)
 *    return optimum
 *
 * The unconstrained optimizer is implemented separately.
 */
public class AugmentedLagrangeOptimizationProblem
{
   private final CostFunction costFunction;
   private final List<ConstraintFunction> inequalityConstraints = new ArrayList<>();
   private final List<ConstraintFunction> equalityConstraints = new ArrayList<>();

   private AugmentedLagrangeConstructor augmentedLagrangeConstructor;

   // ============= Setup =======================

   public AugmentedLagrangeOptimizationProblem(CostFunction costFunction)
   {
      this.costFunction = costFunction;
   }

   /**
    * @param constraint h(x) >= 0
    */
   public void addInequalityConstraint(ConstraintFunction constraint)
   {
      inequalityConstraints.add(constraint);
   }

   /**
    * @param constraint g(x) == 0
    */
   public void addEqualityConstraint(ConstraintFunction constraint)
   {
      equalityConstraints.add(constraint);
   }

   public void clearConstraints()
   {
      equalityConstraints.clear();
      inequalityConstraints.clear();
   }

   /**
    * @param initialPenalty Keep the initial penalty small
    * @param penaltyIncreaseFactor
    */
   public void initialize(double initialPenalty, double penaltyIncreaseFactor)
   {
      augmentedLagrangeConstructor = new AugmentedLagrangeConstructor(initialPenalty,
                                                                      penaltyIncreaseFactor,
                                                                      equalityConstraints.size(),
                                                                      inequalityConstraints.size());
   }

   // ============= Solving =======================

   /**
    * Returns F(x), the unconstrained augmented cost function
    * Use a separate optimizer to solve this unconstrained problem
    */
   public CostFunction getAugmentedCostFunction()
   {
      return this::calculateDualProblemCost;
   }

   /**
    * Evaluates F(x) the unconstrained augmented cost function
    */
   public double calculateDualProblemCost(DMatrixD1 x)
   {
      double objectivesCost = calculateCost(x);
      DMatrixD1 inequalityConstraintEvaluations = evaluateInequalityConstraints(x);
      DMatrixD1 equalityConstraintEvaluations = evaluateEqualityConstraints(x);

      return augmentedLagrangeConstructor.getAugmentedLagrangeCost(objectivesCost, equalityConstraintEvaluations, inequalityConstraintEvaluations);
   }

   /**
    * The unaugmented cost function
    */
   private double calculateCost(DMatrixD1 x)
   {
      return costFunction.calculate(x);
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] >= 0,
    * calculate [h1(x), h2(x), ...] the residual values
    *
    * If all constraints are satisfied, all returned values will be (tolerant) >= to 0.0
    */
   private DMatrixD1 evaluateInequalityConstraints(DMatrixD1 x)
   {
      int numConstraints = inequalityConstraints.size();
      DMatrixRMaj value = new DMatrixRMaj(numConstraints, 1);
      for (int i = 0; i < numConstraints; i++)
      {
         value.set(i, inequalityConstraints.get(i).calculate(x));
      }
      return value;
   }

   /**
    * For constraints G(x) = [g1(x), g2(x), ...] = 0,
    * calculate [g1(x), g2(x), ...] the residual values
    *
    * If all constraints are satisfied, all returned values will be (tolerant) equal to 0.0
    */
   private DMatrixD1 evaluateEqualityConstraints(DMatrixD1 x)
   {
      int numConstraints = equalityConstraints.size();
      DMatrixRMaj value = new DMatrixRMaj(numConstraints, 1);
      for (int i = 0; i < numConstraints; i++)
      {
         value.set(i, equalityConstraints.get(i).calculate(x));
      }
      return new DMatrixRMaj(value);
   }

   public void updateLagrangeMultipliers(DMatrixD1 xOptimal)
   {
      augmentedLagrangeConstructor.updateLagrangeMultipliers(evaluateEqualityConstraints(xOptimal), evaluateInequalityConstraints(xOptimal));
   }

   // ================================ Getter/Setter ========================================

   public void printResults(DMatrixD1 x)
   {
      printResults(x, costFunction.calculate(x), evaluateEqualityConstraints(x), evaluateInequalityConstraints(x));
   }

   public void printResults(DMatrixD1 x, double cost, DMatrixD1 equalityEvaluations, DMatrixD1 inequalityEvaluations)
   {
      LogTools.info("");
      System.out.println("Solution x:");
      for (int i = 0; i < x.getNumElements(); i++)
      {
         LogTools.debug("\t" + x.get(i) + ",");
      }

      System.out.println("Cost f(x):");
      System.out.println("\t" + cost);

      if (equalityEvaluations.getNumElements() > 0)
      {
         System.out.println("Equality Constraints G(x):");
         for (int i = 0; i < equalityEvaluations.getNumElements(); i++)
         {
            System.out.println("\t" + equalityEvaluations.get(i) + " == 0");
         }
      }

      if (inequalityEvaluations.getNumElements() > 0)
      {
         System.out.println("Inquality Constraints H(x):");
         for (int i = 0; i < inequalityEvaluations.getNumElements(); i++)
         {
            System.out.println("\t" + inequalityEvaluations.get(i) + " >= 0");
         }
      }
   }
}
