package us.ihmc.robotics.optimization;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;

import static us.ihmc.robotics.Assert.assertTrue;

/**
 * Unconstrained optimum is (0,0,0)
 * Constrained optimum is (6, 5, 0)
 */
public class TestALM
{
   private AugmentedLagrangeOptimizationProblem augmentedLagrange;
   private DMatrixD1 lagrangeInputVector = new DMatrixRMaj();
   private int numLagrangeIterations = 10;
   private double initialPenalty = 1.0;
   private double penaltyIncreaseFactor = 1.5;

   TDoubleArrayList initial = new TDoubleArrayList(new double[] {10.0, 14.5, 16.0});

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

   // x[2] <= 3
   public static double constraint3(DMatrixD1 inputs)
   {
      return -inputs.get(2) + 3;
   }

   // x[2] <= 3
   public static double constraint4(DMatrixD1 inputs)
   {
      return inputs.get(0) + inputs.get(1) + inputs.get(2) - 6.0;
   }

   public static void convertArrayToMatrix(DMatrixD1 vector, TDoubleArrayList list)
   {
      vector.setData(list.toArray());
      vector.reshape(list.size(), 1);
   }

   public TestALM()
   {
      augmentedLagrange = new AugmentedLagrangeOptimizationProblem(TestALM::costFunction);
      augmentedLagrange.addEqualityConstraint(TestALM::constraint1);
      augmentedLagrange.addInequalityConstraint(TestALM::constraint2);
      augmentedLagrange.addInequalityConstraint(TestALM::constraint3);
//      augmentedLagrange.addEqualityConstraint(TestALM::constraint4);
      augmentedLagrange.initialize(initialPenalty, penaltyIncreaseFactor);

      SingleQueryFunction lagrangeCostFunction = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // convert TDoubleArrayList to DMatrixRMaj
            convertArrayToMatrix(lagrangeInputVector, values);
            double cost = augmentedLagrange.calculateDualProblemCost(lagrangeInputVector);
            return cost;
         }
      };

      // Repeat lagrange step multiple times
      int iteration = 0;
      DMatrixD1 optimumMatrixX = new DMatrixRMaj();
      TDoubleArrayList optimumX = new TDoubleArrayList();
      TDoubleArrayList optimalSolution = new TDoubleArrayList();
      while (iteration < numLagrangeIterations)
      {
         System.out.println("===== Lagrange Iteration: " + iteration + "==========");

         // Run optimization
         if (iteration > 0)
         {
            initial.reset();
            initial.addAll(optimumX);
         }
         GradientDescentModule optimizer = new GradientDescentModule(lagrangeCostFunction, initial);
         optimizer.run();

         // Update lagrange multipliers
         optimumX = optimizer.getOptimalInput();
         convertArrayToMatrix(optimumMatrixX, optimumX);
         augmentedLagrange.updateLagrangeMultipliers(optimumMatrixX);
         iteration += 1;

         optimalSolution = optimizer.getOptimalInput();
         for (int i = 0; i < optimalSolution.size(); i++)
            System.out.println("solution is " + optimalSolution.get(i));

         System.out.println("optimal query is " + optimizer.getOptimalQuery());
      }
      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimalSolution.get(0), 6, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimalSolution.get(1), 5, 1e-3));
      assertTrue("x3 arrived on desired value", MathTools.epsilonCompare(optimalSolution.get(2), 0, 1e-3));

   }


   public static void main(String[] args)
   {
      TestALM alm = new TestALM();

   }
}
