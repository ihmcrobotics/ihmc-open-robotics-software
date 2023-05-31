package us.ihmc.robotics.optimization;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;
import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockAdmmProblem;

import static us.ihmc.robotics.Assert.assertTrue;

public class TestADMM
{
   // -------- This is the first isolated problem --------------

   private int numLagrangeIterations = 10;
   private double initialPenalty = 1.0;
   private double penaltyIncreaseFactor = 1.5;

   TDoubleArrayList initial1 = new TDoubleArrayList(new double[] {10.0});
   TDoubleArrayList initial2 = new TDoubleArrayList(new double[] {10.0});


   // optimum is (0,0,0...)
   public static double costFunction(DMatrixD1 inputs)
   {
      double cost = VectorVectorMult_DDRM.innerProd(inputs, inputs);
      return cost;
   }

   // x[1] == 5
   public static double constraint1(DMatrixD1 inputs)
   {
      return inputs.get(1) - 5;
   }

   // x[0] >= 6
   public static double constraint2(DMatrixD1 inputs)
   {
      return inputs.get(0) - 6;
   }

   public static double blockConstraint1(DMatrixD1... blocks)
   {
      return blocks[0].get(0) + blocks[1].get(0) - 4;
   }


   // -------- This is the second isolated problem --------------

   public static void convertArrayToMatrix(DMatrixD1 vector, TDoubleArrayList list)
   {
      vector.setData(list.toArray());
      vector.reshape(list.size(), 1);
   }

   public static SingleQueryFunction generateBlockAugmentedCostForGD(int index, MultiblockAdmmProblem lagrangeProblem)
   {
      return new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // convert TDoubleArrayList to DMatrixRMaj
            DMatrixD1 lagrangeInputVector = new DMatrixRMaj();
            convertArrayToMatrix(lagrangeInputVector, values);
            return lagrangeProblem.calculateDualCostForBlock(index, lagrangeInputVector); // block2
         }
      };
   }

   public static SingleQueryFunction generateIsolatedAugmentedCostForGD(AugmentedLagrangeOptimizationProblem problem)
   {
      return new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // convert TDoubleArrayList to DMatrixRMaj
            DMatrixD1 lagrangeInputVector = new DMatrixRMaj();
            convertArrayToMatrix(lagrangeInputVector, values);
            return problem.calculateDualProblemCost(lagrangeInputVector);
         }
      };
   }

   public TestADMM()
   {
      AugmentedLagrangeOptimizationProblem augmentedLagrange1 = new AugmentedLagrangeOptimizationProblem(TestALM::costFunction);
      AugmentedLagrangeOptimizationProblem augmentedLagrange2 = new AugmentedLagrangeOptimizationProblem(TestALM::costFunction);

      MultiblockAdmmProblem admm = new MultiblockAdmmProblem();
      admm.addIsolatedProblem(augmentedLagrange1);
      admm.addIsolatedProblem(augmentedLagrange2);
      admm.addEqualityConstraint(TestADMM::blockConstraint1);
      admm.initialize(initialPenalty, penaltyIncreaseFactor);

      AugmentedLagrangeSolver optimizer1;
      AugmentedLagrangeSolver optimizer2;
      DMatrixD1 optimum1 = new DMatrixRMaj();
      DMatrixD1 optimum2 = new DMatrixRMaj();

      // Seed the admm optimization
      SingleQueryFunction isolatedLagrangeCostFunction1 = generateIsolatedAugmentedCostForGD(augmentedLagrange1);
      SingleQueryFunction isolatedLagrangeCostFunction2 = generateIsolatedAugmentedCostForGD(augmentedLagrange2);

      optimizer1 = new AugmentedLagrangeSolver(isolatedLagrangeCostFunction1, initial1, "1");
      optimizer2 = new AugmentedLagrangeSolver(isolatedLagrangeCostFunction2, initial2, "2");

      optimum1 = optimizer1.solveOneRun();
      optimum2 = optimizer2.solveOneRun();

      System.out.println("Initial Seed");
      optimizer1.printOutput();
      optimizer2.printOutput();


      // Do the main lagrange loop
      admm.updateLastOptimalBlocks(optimum1, optimum2);
      SingleQueryFunction lagrangeCostFunction1 = generateBlockAugmentedCostForGD(0, admm);
      SingleQueryFunction lagrangeCostFunction2 = generateBlockAugmentedCostForGD(1, admm);

      int iteration = 0;
      optimizer1 = new AugmentedLagrangeSolver(lagrangeCostFunction1, optimizer1.getOptimumAsArray(), "1");
      optimizer2 = new AugmentedLagrangeSolver(lagrangeCostFunction2, optimizer2.getOptimumAsArray(), "2");
      optimum1 = new DMatrixRMaj();
      optimum2 = new DMatrixRMaj();

      // Repeat lagrange step multiple times
      while (iteration < numLagrangeIterations)
      {
         System.out.println("===== Lagrange Iteration: " + iteration + "==========");

         // Run optimization
         optimum1 = optimizer1.solveOneRun();
         optimum2 = optimizer2.solveOneRun();

         admm.updateLagrangeMultipliers(optimum1, optimum2);
         admm.updateLastOptimalBlocks(optimum1, optimum2);

         iteration += 1;

         optimizer1.printOutput();
         optimizer2.printOutput();
         System.out.println("Constraint: " + blockConstraint1(optimum1, optimum2));
      }

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimum1.get(0), 2, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimum2.get(0), 2, 1e-3));

   }


   public static void main(String[] args)
   {
      TestADMM alm = new TestADMM();

   }
}
