//package us.ihmc.robotics.optimization;
//
//import gnu.trove.list.array.TDoubleArrayList;
//import org.ejml.data.DMatrixD1;
//import org.ejml.data.DMatrixRMaj;
//import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
//import us.ihmc.commons.MathTools;
//import us.ihmc.robotics.numericalMethods.GradientDescentModule;
//import us.ihmc.robotics.numericalMethods.SingleQueryFunction;
//import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;
//import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockADMM;
//
//import static us.ihmc.robotics.Assert.assertTrue;
//
//public class TestADMM
//{
//   // -------- This is the first isolated problem --------------
//
//   private int numLagrangeIterations = 10;
//   private double initialPenalty = 1.0;
//   private double penaltyIncreaseFactor = 1.5;
//
//   TDoubleArrayList initial = new TDoubleArrayList(new double[] {10.0, 14.5, 16.0});
//
//   // optimum is (0,0,0...)
//   public static double costFunction(DMatrixD1 inputs)
//   {
//      double cost = VectorVectorMult_DDRM.innerProd(inputs, inputs);
//      return cost;
//   }
//
//   // x[1] == 5
//   public static double constraint1(DMatrixD1 inputs)
//   {
//      return inputs.get(1) - 5;
//   }
//
//   // x[0] >= 6
//   public static double constraint2(DMatrixD1 inputs)
//   {
//      return inputs.get(0) - 6;
//   }
//
//   // x[2] <= 3
//   public static double constraint3(DMatrixD1 inputs)
//   {
//      return -inputs.get(2) + 3;
//   }
//
//   public static double blockConstraint1(DMatrixD1... blocks)
//   {
//      return blocks[0].get(0) + blocks[1].get(0) - 4;
//   }
//
//
//   // -------- This is the second isolated problem --------------
//
//   public static void convertArrayToMatrix(DMatrixD1 vector, TDoubleArrayList list)
//   {
//      vector.setData(list.toArray());
//      vector.reshape(list.size(), 1);
//   }
//
//   public static SingleQueryFunction generateAugmentedCostForGD(int index, MultiblockADMM lagrangeProblem)
//   {
//      return new SingleQueryFunction()
//      {
//         @Override
//         public double getQuery(TDoubleArrayList values)
//         {
//            // convert TDoubleArrayList to DMatrixRMaj
//            DMatrixD1 lagrangeInputVector = new DMatrixRMaj();
//            convertArrayToMatrix(lagrangeInputVector, values);
//            return lagrangeProblem.calculateDualCost(index,
//                                                     new DMatrixRMaj(new double[] {values.get(0)}),
//                                                     new DMatrixRMaj(new double[] {values.get(1)}));
//         }
//      };
//   }
//
//   public TestADMM()
//   {
//      AugmentedLagrangeOptimizationProblem augmentedLagrange1 = new AugmentedLagrangeOptimizationProblem(TestALM::costFunction);
//      augmentedLagrange1.initialize(initialPenalty, penaltyIncreaseFactor);
//
//      AugmentedLagrangeOptimizationProblem augmentedLagrange2 = new AugmentedLagrangeOptimizationProblem(TestALM::costFunction);
//      augmentedLagrange1.initialize(initialPenalty, penaltyIncreaseFactor);
//
//      MultiblockADMM admm = new MultiblockADMM();
//      admm.addEqualityConstraint(TestADMM::blockConstraint1);
//
//      SingleQueryFunction lagrangeCostFunction1 = generateAugmentedCostForGD(0, admm);
//      SingleQueryFunction lagrangeCostFunction2 = generateAugmentedCostForGD(1, admm);
//
//      // Repeat lagrange step multiple times
//      int iteration = 0;
//      DMatrixD1 optimumMatrixX = new DMatrixRMaj();
//      TDoubleArrayList optimumX = new TDoubleArrayList();
//      TDoubleArrayList optimalSolution = new TDoubleArrayList();
//      while (iteration < numLagrangeIterations)
//      {
//         System.out.println("===== Lagrange Iteration: " + iteration + "==========");
//
//         // Run optimization
//         if (iteration > 0)
//         {
//            initial.reset();
//            initial.addAll(optimumX);
//         }
//         GradientDescentModule optimizer1 = new GradientDescentModule(lagrangeCostFunction1, initial);
//         optimizer1.run();
//
//         // Update lagrange multipliers
//         optimumX = optimizer1.getOptimalInput();
//         convertArrayToMatrix(optimumMatrixX, optimumX);
//         admm.updateLagrangeMultipliers(optimumMatrixX);
//
//
//         iteration += 1;
//
//         optimalSolution = optimizer.getOptimalInput();
//         for (int i = 0; i < optimalSolution.size(); i++)
//            System.out.println("solution is " + optimalSolution.get(i));
//
//         System.out.println("optimal query is " + optimizer.getOptimalQuery());
//      }
//      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimalSolution.get(0), 6, 1e-3));
//      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimalSolution.get(1), 5, 1e-3));
//      assertTrue("x3 arrived on desired value", MathTools.epsilonCompare(optimalSolution.get(2), 0, 1e-3));
//
//   }
//
//
//   public static void main(String[] args)
//   {
//      TestADMM alm = new TestADMM();
//
//   }
//}
