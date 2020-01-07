package us.ihmc.robotics.linearAlgebra.cdreSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARETools;

public class NumericCDRESolver extends AbstractCDRESolver
{
   private static final double defaultDt = 1e-4;
   private final double dt;

   private double finalTime;

   private final DenseMatrix64F PFinal = new DenseMatrix64F(0, 0);

   private final RecyclingArrayList<DenseMatrix64F> PTrajectory = new RecyclingArrayList<>(() -> new DenseMatrix64F(0, 0));

   public NumericCDRESolver()
   {
      this(defaultDt);
   }

   public NumericCDRESolver(double dt)
   {
      this.dt = dt;
   }

   public void setFinalBoundaryCondition(double finalTime, DenseMatrix64F PFinal)
   {
      this.PFinal.set(PFinal);
      this.finalTime = finalTime;

      PTrajectory.clear();
   }

   private final DenseMatrix64F PDot = new DenseMatrix64F(0, 0);

   // just does a really simple first order reverse time integration.
   public void computePFunction(double initialTime)
   {
      double time = finalTime;

      DenseMatrix64F lastP = PTrajectory.add();
      lastP.set(PFinal);
      time -= dt;

      PDot.reshape(PFinal.numRows, PFinal.numCols);

      for (; time >= initialTime; time -= dt)
      {
         CARETools.computeRiccatiRate(lastP, A, Q, M, PDot);

         DenseMatrix64F nextP = PTrajectory.add();
         nextP.set(lastP);
         CommonOps.addEquals(nextP, -dt, PDot);

         lastP = nextP;
      }
   }

   private final DenseMatrix64F PToReturn = new DenseMatrix64F(0, 0);

   @Override
   public DenseMatrix64F getP(double time)
   {
      int index = (int) (Math.floor(time / dt));
      index = MathTools.clamp(index, 0, PTrajectory.size() - 2);

      double interpolationFraction = (time - index * dt) / dt;
      int reverseTimeTrajectoryIndex = PTrajectory.size() - 1 - index;

      PToReturn.set(PTrajectory.get(reverseTimeTrajectoryIndex));
      CommonOps.scale(1.0 - interpolationFraction, PToReturn);
      CommonOps.addEquals(PToReturn, interpolationFraction, PTrajectory.get(reverseTimeTrajectoryIndex - 1));

      return PToReturn;
   }

}
