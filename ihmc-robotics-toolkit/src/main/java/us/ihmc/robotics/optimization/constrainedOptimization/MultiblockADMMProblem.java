package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.optimization.CostFunction;

import java.util.ArrayList;
import java.util.List;

/**
 * reference: https://arxiv.org/pdf/1802.09592.pdf
 * Solves the optimization problem for blocks of parameters x[] = [x1, x2, ...]:
 *    minimize f1(x1) + f2(x2) + ...
 *    st:
 *       G1[](x1) == 0, G2[](x2) == 0, ...
 *       H1[](x1) >= 0, H2[](x2) >= 0, ...
 *
 *       J[](x1, x2,...) == 0
 *       K[](x1, x2,...) >= 0
 * ------
 * which can be considered the isolated subproblems for each block xi:
 *    minimize fi(xi)
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
 *
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
public class MultiblockADMMProblem
{
   private final List<AugmentedLagrangeOptimizationProblem> isolatedOptimizationProblems = new ArrayList<>(); // size = numblocks
   private final List<BlockConstraintFunction> inequalityConstraints = new ArrayList<>();
   private final List<BlockConstraintFunction> equalityConstraints = new ArrayList<>();

   private AugmentedLagrangeConstructor multiblockAugmentedLagrangeConstructor;
   private DMatrixD1[] optimalBlocksFromLastIteration;

   // =========================== Setup ===================================

   public MultiblockADMMProblem()
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

   public void clearConstraints()
   {
      equalityConstraints.clear();
      inequalityConstraints.clear();
   }

   public void clearIsolatedProblems()
   {
      this.isolatedOptimizationProblems.clear();
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

   // ======================== Solving ===================================

   /**
    * Returns Li(xi) == Li(xi, x_other*), the unconstrained augmented cost function
    * x_other* is stored implicitely in this class by updating {@link #saveOptimalBlocksForLastIteration(DMatrixD1[])}
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
            return calculateDualCostForBlock(blockIndex, x, optimalBlocksFromLastIteration);
         }
      };
   }

   /**
    * evaluates Li(xi) == Li(xi, x_other*)
    * @param blockIndex i
    * @param x xi
    */
   public double calculateDualCostForBlock(int blockIndex, DMatrixD1 x, DMatrixD1[] optimalBlocksForLastIteration)
   {
      if (optimalBlocksForLastIteration.length != getNumBlocks())
         throw new RuntimeException(
               "Not enough blocks " + optimalBlocksForLastIteration.length + " were provided for all blocks of the problem " + getNumBlocks());
      DMatrixD1[] blocksCopy = optimalBlocksForLastIteration.clone();
      blocksCopy[blockIndex] = x;
      return calculateDualCostForBlock(blockIndex, blocksCopy);
   }

   /**
    * evaluates Li(xi, x_other*)
    * @param blockIndex i
    * @param blocks x*, which contains [xi*, x_other*] in normal order [x1*, x2*, ...]
    */
   private double calculateDualCostForBlock(int blockIndex, DMatrixD1[] blocks)
   {
      double isolatedCost = isolatedOptimizationProblems.get(blockIndex).calculateDualProblemCost(blocks[blockIndex]);
      DMatrixD1 inequalityConstraintEvaluations = evaluateGlobalInequalityConstraints(blocks);
      DMatrixD1 equalityConstraintEvaluations = evaluateEqualityConstraints(blocks);

      return multiblockAugmentedLagrangeConstructor.getAugmentedLagrangeCost(isolatedCost, equalityConstraintEvaluations, inequalityConstraintEvaluations);
   }

   /**
    * For constraints K(x[]) = [k1(x[]), k2(x[]), ...] >= 0,
    * calculate [k1(x[]), k2(x[]), ...] the residual values
    *
    * If all constraints are satisfied, all returned values will be (tolerant) >= to 0.0
    */
   private DMatrixD1 evaluateGlobalInequalityConstraints(DMatrixD1[] blocks)
   {
      int numConstraints = inequalityConstraints.size();
      DMatrixD1 value = new DMatrixRMaj(numConstraints, 1);
      for (int i = 0; i < numConstraints; i++)
      {
         value.set(i, inequalityConstraints.get(i).calculate(blocks));
      }
      return value;
   }

   /**
    * For constraints J(x[]) = [j1(x[]), j2(x[]), ...] = 0,
    * calculate [j1(x[]), j2(x[]), ...] the residual values
    *
    * If all constraints are satisfied, all returned values will be (tolerant) equal to 0.0
    */
   private DMatrixD1 evaluateEqualityConstraints(DMatrixD1[] blocks)
   {
      int numConstraints = equalityConstraints.size();
      DMatrixD1 value = new DMatrixRMaj(numConstraints, 1);
      for (int i = 0; i < numConstraints; i++)
      {
         value.set(i, equalityConstraints.get(i).calculate(blocks));
      }
      return value;
   }

   public void updateLagrangeMultipliers(DMatrixD1[] optimalBlocks)
   {
      if (optimalBlocks.length != getNumBlocks())
         throw new RuntimeException(
               "Not enough values " + optimalBlocks.length + " were provided for all blocks of the problem " + getNumBlocks());

      multiblockAugmentedLagrangeConstructor.updateLagrangeMultipliers(evaluateEqualityConstraints(optimalBlocks),
                                                                       evaluateGlobalInequalityConstraints(optimalBlocks));

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
   public void saveOptimalBlocksForLastIteration(DMatrixD1[] optimalBlocks)
   {
      if (optimalBlocks.length != getNumBlocks())
         throw new RuntimeException(
               "Not enough values " + optimalBlocks.length + " were provided for all blocks of the problem " + getNumBlocks());

      optimalBlocksFromLastIteration = optimalBlocks;
   }

   // ================================ Getter/Setter ========================================

   public List<AugmentedLagrangeOptimizationProblem> getIsolatedOptimizationProblems()
   {
      return isolatedOptimizationProblems;
   }

   public int getNumBlocks()
   {
      return isolatedOptimizationProblems.size();
   }

   public void printResults(DMatrixD1[] blocks)
   {
      printResults(evaluateEqualityConstraints(blocks), evaluateGlobalInequalityConstraints(blocks), blocks);
   }

   public void printResults(DMatrixD1 equalityEvaluations, DMatrixD1 inequalityEvaluations, DMatrixD1[] blocks)
   {
      LogTools.info("");
      for (int i = 0; i < isolatedOptimizationProblems.size(); i++)
      {
         System.out.println("-- Isolated Problem " + i + ": --");
         isolatedOptimizationProblems.get(i).printResults(blocks[i]);
      }

      if (equalityEvaluations.getNumElements() > 0)
      {
         System.out.println("-- Global Equality Constraints J(x[]): --");
         for (int i = 0; i < equalityEvaluations.getNumElements(); i++)
         {
            System.out.println("\t" + equalityEvaluations.get(i) + " == 0");
         }
      }

      if (inequalityEvaluations.getNumElements() > 0)
      {
         System.out.println("-- Global Inquality Constraints K(x[]): --");
         for (int i = 0; i < inequalityEvaluations.getNumElements(); i++)
         {
            System.out.println("\t" + inequalityEvaluations.get(i) + " >= 0");
         }
      }
   }
}