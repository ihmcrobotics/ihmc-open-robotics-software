package us.ihmc.robotics.optimization;

import java.util.LinkedHashSet;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

/**
* @author twan
*         Date: 8/9/13
*/
class ActiveSearchSolutionInfo
{
   private final DenseMatrix64F solution = new DenseMatrix64F(1, 1);
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

   public void setSolution(DenseMatrix64F solution)
   {
      this.solution.set(solution);
   }

   public void clearActiveSet()
   {
      this.activeSet.clear();
   }

   public DenseMatrix64F getSolution()
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
