package us.ihmc.systemidentification;

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

      UnconstrainedLeastSquares optimizer = new FactoryOptimization().leastSquaresLM(1e-3, true);
      optimizer.setFunction(residual, null);
      
      double[] prm = new double[residual.getN()];
      residual.getCurrentLinkCom().set(prm);
      
      optimizer.initialize(prm, 1e-12, 1e-12);
      
      for(int i=0;i<5;i++)
      {
         optimizer.iterate();
         prm = optimizer.getParameters();
         System.out.println("iter " + i + " obj: "+optimizer.getFunctionValue());
      }
   }
}
