package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;

import java.util.ArrayList;
import java.util.List;

/**
 * TODO: generalize with {@link AugmentedLagrangeOptimizationProblem}
 *
 * Solves the optimization problem:
 *    minimize f1(x1) + f2(x2) + ...
 *    st: Gi(xi,...) >= 0 for G[]
 *       Hi(xi,...) == 0 for H[]
 *    where xi are vectors ("blocks")
 *
 */
public class MultiblockAdmmProblem
{
   private List<AugmentedLagrangeOptimizationProblem> isolatedOptimizationProblems = new ArrayList<>(); // size = numblocks
   List<BlockConstraintFunction> inequalityConstraints = new ArrayList<>();
   List<BlockConstraintFunction> equalityConstraints = new ArrayList<>();

   private AugmentedLagrangeConstructor multiblockAugmentedLagrangeConstructor;

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

   public double calculateDualCost(int blockIndex, DMatrixRMaj... blocks)
   {
      double isolatedCost = isolatedOptimizationProblems.get(blockIndex).calculateDualProblemCost(blocks[blockIndex]);
      DMatrixD1 inequalityConstraintEvaluations = calculateInequalityConstraintVector(blocks);
      DMatrixD1 equalityConstraintEvaluations = calculateEqualityConstraintVector(blocks);

      return multiblockAugmentedLagrangeConstructor.getAugmentedLagrangeCost(isolatedCost,
                                                                   equalityConstraintEvaluations,
                                                                   inequalityConstraintEvaluations);
   }

   public void updateLagrangeMultipliers(DMatrixRMaj... optimalBlocks)
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
    * For constraints G(x) = [g1(x), g2(x), ...] >= 0, calculate [g1(x), g2(x), ...]
    * @return
    */
   private DMatrixD1 calculateInequalityConstraintVector(DMatrixRMaj... blocks)
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
   private DMatrixD1 calculateEqualityConstraintVector(DMatrixRMaj... blocks)
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
