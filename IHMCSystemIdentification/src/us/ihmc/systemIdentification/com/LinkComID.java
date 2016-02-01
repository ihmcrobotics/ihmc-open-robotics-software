package us.ihmc.systemIdentification.com;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;

public class LinkComID
{
   ComCopResidual residual;

   public LinkComID(ComCopResidual residual)
   {
      this.residual = residual;
   }

   public void optimize()
   {
      new FactoryOptimization();
      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);
      optimizer.setFunction(residual, null);

      double[] prm = new double[residual.getNumOfInputsN()];
      residual.getCurrentLinkCom().set(prm);

      optimizer.initialize(prm, 0, 0);
      boolean converged;

      int maxIter = 10;
      for (int i = 0; i < maxIter; i++)
      {
         converged = optimizer.iterate();
         prm = optimizer.getParameters();
         System.out.println("iter " + i + " obj: " + optimizer.getFunctionValue() + "converged " + converged);

         if (optimizer.isConverged())
            break;

      }
      System.out.println("Optimiztion finished.");

   }
}
