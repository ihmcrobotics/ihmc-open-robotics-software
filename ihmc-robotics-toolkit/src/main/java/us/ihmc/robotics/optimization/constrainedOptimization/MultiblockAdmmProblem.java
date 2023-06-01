package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;

import java.util.ArrayList;
import java.util.List;

/**
 * Solves the optimization problem:
 *    minimize f1(x1) + f2(x2) + ...
 *    st:
 *       G1[](x1) == 0, G2[](x2) == 0, ...
 *       H1[](x1) >= 0, H2[](x2) >= 0, ...
 *
 *       J[](x1, x2,...) == 0
 *       K[](x1, x2,...) >= 0
 * ------
 * which can be considered the isolated blocks / subproblems:
 *    minimize fi(i)
 *    st:
 *       Gi[](xi) == 0
 *       Hi[](xi) >= 0
 * that must satisfy the global constraints
 *    J[](x1, x2,...) == 0
 *    K[](x1, x2,...) >= 0
 * ------
 * We solve the constrained problems using the augmented lagrange method by creating
 * for each isolated problem the augmented cost:
 *    Fi(xi) = fi(xi) + augmentations(Hi[](xi), Gi[](xi))
 * the global augmented cost that incorporates global constraints:
 *    Li(xi, x_other*) = Fi(xi) + augmentations(J[](xi, x_other*), K[](xi, x_other*))
 *
 * The global constraints are solved using the Alternating Direction Method of Multipliers
 * within {@link MultiblockADMMOptimizer} through the algorithm
 *    for each block xi,
 *       initialize xi* = optimize(Fi(xi)) for xi
 *
 *    for n iterations
 *       for each block xi,
 *          solve xi* = optimize(Li(xi, x_other*...)) for xi
 *       update the lagrange multipliers with x*
 *    return x*
 * ------
 * The optimizers for the unconstrained optimization Fi(xi) and Li(xi, x_other) are implemented separately.
 */
public class MultiblockAdmmProblem
{
   private List<AugmentedLagrangeOptimizationProblem> isolatedOptimizationProblems = new ArrayList<>(); // size = numblocks
   List<BlockConstraintFunction> inequalityConstraints = new ArrayList<>();
   List<BlockConstraintFunction> equalityConstraints = new ArrayList<>();

   private AugmentedLagrangeConstructor multiblockAugmentedLagrangeConstructor;
   private DMatrixD1[] lastOptimalBlocks;

   public MultiblockAdmmProblem()
   {
   }

   /**
    * An isolated block / subproblem:
    *    minimize fi(i)
    *    st:
    *       Gi[](xi) == 0
    *       Hi[](xi) >= 0
    */
   public void addIsolatedProblem(AugmentedLagrangeOptimizationProblem isolatedProblem)
   {
      isolatedOptimizationProblems.add(isolatedProblem);
   }

   /**
    * A global constraint
    *    K(x1, x2,...) >= 0
    */
   public void addInequalityConstraint(BlockConstraintFunction constraint)
   {
      inequalityConstraints.add(constraint);
   }

   /**
    * A global constraint
    *    J(x1, x2,...) == 0
    */
   public void addEqualityConstraint(BlockConstraintFunction constraint)
   {
      equalityConstraints.add(constraint);
   }

   public void initialize(double penalty, double penaltyIncreaseFactor)
   {
      for (AugmentedLagrangeOptimizationProblem problem : isolatedOptimizationProblems)
      {
         problem.initialize(penalty, penaltyIncreaseFactor);
      }

      multiblockAugmentedLagrangeConstructor = new AugmentedLagrangeConstructor(penalty,
                                                                                penaltyIncreaseFactor,
                                                                                equalityConstraints.size(),
                                                                                inequalityConstraints.size());
   }

   /**
    * Returns Li(xi) == Li(xi, x_other*), the unconstrained augmented cost function
    * x_other* is stored implicitely in this class by updating {@link #updateLastOptimalBlocks(DMatrixD1...)}
    *
    * Use a separate optimizer to solve this unconstrained problem
    */
   public CostFunction getAugmentedCostFunctionForBlock(int blockIndex)
   {
      return new CostFunction()
      {
         @Override
         public double calculate(DMatrixD1 x)
         {
            return calculateDualCostForBlock(blockIndex, x);
         }
      };
   }

   /**
    * evaluates Li(xi) == Li(xi, x_other*)
    * @param blockIndex i
    * @param x xi
    */
   public double calculateDualCostForBlock(int blockIndex, DMatrixD1 x)
   {
      DMatrixD1[] blocksCopy = lastOptimalBlocks.clone();
      blocksCopy[blockIndex] = x;
      return calculateDualCostForBlock(blockIndex, blocksCopy);
   }

   /**
    * evaluates Li(xi, x_other*)
    * @param blockIndex i
    * @param lastOptimalblocks x*, which contains [xi*, x_other*] in normal order [x1*, x2*, ...]
    */
   private double calculateDualCostForBlock(int blockIndex, DMatrixD1... lastOptimalblocks)
   {
      double isolatedCost = isolatedOptimizationProblems.get(blockIndex).calculateDualProblemCost(lastOptimalblocks[blockIndex]);
      DMatrixD1 inequalityConstraintEvaluations = calculateInequalityConstraintVector(lastOptimalblocks);
      DMatrixD1 equalityConstraintEvaluations = calculateEqualityConstraintVector(lastOptimalblocks);

      return multiblockAugmentedLagrangeConstructor.getAugmentedLagrangeCost(isolatedCost,
                                                                   equalityConstraintEvaluations,
                                                                   inequalityConstraintEvaluations);
   }

   /**
    * For constraints G(x) = [g1(x), g2(x), ...] >= 0, calculate [g1(x), g2(x), ...]
    * @return
    */
   private DMatrixD1 calculateInequalityConstraintVector(DMatrixD1... blocks)
   {
      int numConstraints = inequalityConstraints.size();
      double[] value = new double[numConstraints];
      for (int i = 0; i < numConstraints; i++)
      {
         value[i] = inequalityConstraints.get(i).calculate(blocks);
      }
      return new DMatrixRMaj(value);
   }

   /**
    * For constraints H(x) = [h1(x), h2(x), ...] = 0, calculate [h1(x), h2(x), ...]
    * @return
    */
   private DMatrixD1 calculateEqualityConstraintVector(DMatrixD1... blocks)
   {
      int numConstraints = equalityConstraints.size();
      double[] value = new double[numConstraints];
      for (int i = 0; i < numConstraints; i++)
      {
         value[i] = equalityConstraints.get(i).calculate(blocks);
      }
      return new DMatrixRMaj(value);
   }

   public void updateLagrangeMultipliers(DMatrixD1... optimalBlocks)
   {
      multiblockAugmentedLagrangeConstructor.updateLagrangeMultipliers(calculateEqualityConstraintVector(optimalBlocks),
                                                                       calculateInequalityConstraintVector(optimalBlocks));

      for (int i = 0; i < isolatedOptimizationProblems.size(); i++)
      {
         AugmentedLagrangeOptimizationProblem problem = isolatedOptimizationProblems.get(i);
         // rho = rho + p * c_i(x'_i)
         problem.updateLagrangeMultipliers(optimalBlocks[i]);
      }
   }

   /**
    * Saves the optimal blocks from the last iteration x* = [x1*, x2*, ...]
    */
   public void updateLastOptimalBlocks(DMatrixD1... optimalBlocks)
   {
      lastOptimalBlocks = optimalBlocks;
   }

   public List<AugmentedLagrangeOptimizationProblem> getIsolatedOptimizationProblems()
   {
      return isolatedOptimizationProblems;
   }

   public int getNumBlocks()
   {
      return isolatedOptimizationProblems.size();
   }
}
