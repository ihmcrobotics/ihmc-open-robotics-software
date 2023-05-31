package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;

import java.util.ArrayList;
import java.util.List;

/**
 * Solves the optimization problem:
 *    minimize f1(x1) + f2(x2) + ...
 *    st:
 *
 *       Gi(xi,...) >= 0 for G[]
 *       Hi(xi,...) == 0 for H[]
 *    where xi are vectors ("blocks")
 *
 * The actual optimizer is implemented separately.
 *
 * An example use case is: TODO
 *    ALOP lagrangian = new ALOP(costFunction, inequalityConstraints, equalityConstraints)
 *    CostFunction augmentedCostFunction = lagrangian.calculateDualProblemCost
 *
 *    Optimizer optimizer = new Optimizer(augmentedCostFunction)
 *    for (n lagrangian steps)
 *       optimum = optimizer.doRun()
 *       lagrangian.updateLagrangeMultipliers(optimum)
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

   public void addIsolatedProblem(AugmentedLagrangeOptimizationProblem isolatedProblem)
   {
      isolatedOptimizationProblems.add(isolatedProblem);
   }

   public void addInequalityConstraint(BlockConstraintFunction constraint)
   {
      inequalityConstraints.add(constraint);
   }

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

   public double calculateDualCostForBlock(int blockIndex, DMatrixD1 x)
   {
      DMatrixD1[] blocksCopy = lastOptimalBlocks.clone();
      blocksCopy[blockIndex] = x;
      return calculateDualCostForBlock(blockIndex, blocksCopy);
   }

   public double calculateDualCostForBlock(int blockIndex, DMatrixD1... blocks)
   {
      double isolatedCost = isolatedOptimizationProblems.get(blockIndex).calculateDualProblemCost(blocks[blockIndex]);
      DMatrixD1 inequalityConstraintEvaluations = calculateInequalityConstraintVector(blocks);
      DMatrixD1 equalityConstraintEvaluations = calculateEqualityConstraintVector(blocks);

      return multiblockAugmentedLagrangeConstructor.getAugmentedLagrangeCost(isolatedCost,
                                                                   equalityConstraintEvaluations,
                                                                   inequalityConstraintEvaluations);
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

   public void updateLastOptimalBlocks(DMatrixD1... optimalBlocks)
   {
      lastOptimalBlocks = optimalBlocks;
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
}
