package us.ihmc.robotics.optimization;

import java.util.LinkedHashSet;
import java.util.Set;

import org.ejml.data.DMatrixRMaj;

/**
* @author twan
*         Date: 8/9/13
*/
class ActiveSearchSolutionInfo
{
   private final DMatrixRMaj solution = new DMatrixRMaj(1, 1);
   private boolean converged;
   private int iterations;
   private final Set<Integer> activeSet = new LinkedHashSet<Integer>();

   public void reset(int solutionSize)
   {
      solution.reshape(solutionSize, 1);
      converged = false;
      iterations = 0;
      activeSet.clear();
   }

   public void setSolution(DMatrixRMaj solution)
   {
      this.solution.set(solution);
   }

   public void clearActiveSet()
   {
      this.activeSet.clear();
   }

   public DMatrixRMaj getSolution()
   {
      return solution;
   }

   public int getIterations()
   {
      return iterations;
   }

   public Set<Integer> getActiveSet()
   {
      return activeSet;
   }

   public boolean isConverged()
   {
      return converged;
   }

   void setConverged(boolean converged)
   {
      this.converged = converged;
   }

   public void incrementIterations()
   {
      iterations++;
   }
}
