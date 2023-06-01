package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;

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
   private CostFunction costFunction;
   private final List<ConstraintFunction> inequalityConstraints = new ArrayList<>();
   private final List<ConstraintFunction> equalityConstraints = new ArrayList<>();

   private AugmentedLagrangeConstructor augmentedLagrangeConstructor;

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

   /**
    * Returns F(x), the unconstrained augmented cost function
    * Use a separate optimizer to solve this unconstrained problem
    */
   public CostFunction getAugmentedCostFunction()
   {
      return new CostFunction()
      {
         @Override
         public double calculate(DMatrixD1 x)
         {
            return calculateDualProblemCost(x);
         }
      };
   }

   /**
    * Evaluates F(x) the unconstrained augmented cost function
    */
   public double calculateDualProblemCost(DMatrixD1 x)
   {
      double objectivesCost = calculateCost(x);
      DMatrixD1 inequalityConstraintEvaluations = evaluateInequalityConstraints(x);
      DMatrixD1 equalityConstraintEvaluations = evaluateEqualityConstraints(x);

      return augmentedLagrangeConstructor.getAugmentedLagrangeCost(objectivesCost,
                                                                   equalityConstraintEvaluations,
                                                                   inequalityConstraintEvaluations);
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
    * calculate [h1(x), h2(x), ...]
    */
   private DMatrixD1 evaluateInequalityConstraints(DMatrixD1 x)
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
    * For constraints G(x) = [g1(x), g2(x), ...] = 0,
    * calculate [g1(x), g2(x), ...]
    */
   private DMatrixD1 evaluateEqualityConstraints(DMatrixD1 x)
   {
      int numConstraints = equalityConstraints.size();
      double[] value = new double[numConstraints];
      for (int i = 0; i < numConstraints; i++)
      {
         value[i] = equalityConstraints.get(i).calculate(x);
      }
      return new DMatrixRMaj(value);
   }


   public void updateLagrangeMultipliers(DMatrixD1 xOptimal)
   {
      augmentedLagrangeConstructor.updateLagrangeMultipliers(evaluateEqualityConstraints(xOptimal),
                                                             evaluateInequalityConstraints(xOptimal));
   }


}
