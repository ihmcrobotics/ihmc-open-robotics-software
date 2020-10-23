package us.ihmc.systemIdentification.com;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;

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
      UnconstrainedLeastSquares optimizer = FactoryOptimization.levenbergMarquardt(new ConfigLevenbergMarquardt(), true);
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
