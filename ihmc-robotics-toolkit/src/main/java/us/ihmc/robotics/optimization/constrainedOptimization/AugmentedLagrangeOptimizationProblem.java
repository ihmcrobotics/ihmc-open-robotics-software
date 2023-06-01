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
   private final List<ConstraintFunction> inequalityConstraints = new ArrayList<>();
   private final List<ConstraintFunction> equalityConstraints = new ArrayList<>();

   private AugmentedLagrangeConstructor augmentedLagrangeConstructor;

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
      augmentedLagrangeConstructor = new AugmentedLagrangeConstructor(initialPenalty,
                                                                      penaltyIncreaseFactor,
                                                                      equalityConstraints.size(),
                                                                      inequalityConstraints.size());
   }

   public double calculateDualProblemCost(DMatrixD1 x)
   {
      double objectivesCost = calculateObjectives(x);
      DMatrixD1 inequalityConstraintEvaluations = calculateInequalityConstraintVector(x);
      DMatrixD1 equalityConstraintEvaluations = calculateEqualityConstraintVector(x);

      return augmentedLagrangeConstructor.getAugmentedLagrangeCost(objectivesCost,
                                                                   equalityConstraintEvaluations,
                                                                   inequalityConstraintEvaluations);
   }

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

   public void updateLagrangeMultipliers(DMatrixD1 xOptimal)
   {
      augmentedLagrangeConstructor.updateLagrangeMultipliers(calculateEqualityConstraintVector(xOptimal),
                                                             calculateInequalityConstraintVector(xOptimal)
                                                             );
   }

   private double calculateObjectives(DMatrixD1 x)
   {
      return costFunction.calculate(x);
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] >= 0, calculate [h1(x), h2(x), ...]
    */
   private DMatrixD1 calculateInequalityConstraintVector(DMatrixD1 x)
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
    * For constraints H(x) = [h1(x), h2(x), ...] = 0, calculate [h1(x), h2(x), ...]
    * @return
    */
   private DMatrixD1 calculateEqualityConstraintVector(DMatrixD1 x)
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
