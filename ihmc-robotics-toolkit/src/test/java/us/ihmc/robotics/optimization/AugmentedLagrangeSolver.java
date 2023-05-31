package us.ihmc.robotics.optimization;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class AugmentedLagrangeSolver
{
   SingleQueryFunction lagrangeCostFunction;
   int iteration = 0;
   DMatrixD1 optimumMatrixX = new DMatrixRMaj();
   TDoubleArrayList optimumX = new TDoubleArrayList();
   TDoubleArrayList initial = new TDoubleArrayList(); // do a copu
   String name;

   double optimumCost;

   public AugmentedLagrangeSolver(SingleQueryFunction lagrangeCostFunction, TDoubleArrayList initial, String name)
   {
      this.lagrangeCostFunction = lagrangeCostFunction;
      this.initial.addAll(initial);
      this.name = name;
   }



   public DMatrixD1 solveOneRun()
   {
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

      iteration += 1;

      optimumCost = optimizer.getOptimalQuery();
      return optimumMatrixX;
   }

   public double getOptimumCost()
   {
      return optimumCost;
   }
   public TDoubleArrayList getOptimumAsArray()
   {
      return optimumX;
   }

   public static void convertArrayToMatrix(DMatrixD1 vector, TDoubleArrayList list)
   {
      vector.setData(list.toArray());
      vector.reshape(list.size(), 1);
   }

   public void printOutput()
   {
      for (int i = 0; i < optimumMatrixX.getNumElements(); i++)
         System.out.println(name + ": solution is " + optimumMatrixX.get(i));
      System.out.println("optimal cost is " + getOptimumCost());
   }
}
