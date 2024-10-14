package us.ihmc.robotics.linearAlgebra.cdreSolvers;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARETools;

public class NumericCDRESolver extends AbstractCDRESolver
{
   private static final double defaultDt = 1e-4;
   private final double dt;

   private double finalTime;

   private final DMatrixRMaj PFinal = new DMatrixRMaj(0, 0);

   private final RecyclingArrayList<DMatrixRMaj> PTrajectory = new RecyclingArrayList<>(() -> new DMatrixRMaj(0, 0));

   public NumericCDRESolver()
   {
      this(defaultDt);
   }

   public NumericCDRESolver(double dt)
   {
      this.dt = dt;
   }

   public void setFinalBoundaryCondition(double finalTime, DMatrixRMaj PFinal)
   {
      this.PFinal.set(PFinal);
      this.finalTime = finalTime;

      PTrajectory.clear();
   }

   private final DMatrixRMaj PDot = new DMatrixRMaj(0, 0);

   // just does a really simple first order reverse time integration.
   public void computePFunction(double initialTime)
   {
      double time = finalTime;

      DMatrixRMaj lastP = PTrajectory.add();
      lastP.set(PFinal);
      time -= dt;

      PDot.reshape(PFinal.numRows, PFinal.numCols);

      for (; time >= initialTime; time -= dt)
      {
         CARETools.computeRiccatiRate(lastP, A, Q, M, PDot);

         DMatrixRMaj nextP = PTrajectory.add();
         nextP.set(lastP);
         CommonOps_DDRM.addEquals(nextP, -dt, PDot);

         lastP = nextP;
      }
   }

   private final DMatrixRMaj PToReturn = new DMatrixRMaj(0, 0);

   @Override
   public DMatrixRMaj getP(double time)
   {
      int index = (int) (Math.floor(time / dt));
      index = MathTools.clamp(index, 0, PTrajectory.size() - 2);

      double interpolationFraction = (time - index * dt) / dt;
      int reverseTimeTrajectoryIndex = PTrajectory.size() - 1 - index;

      PToReturn.set(PTrajectory.get(reverseTimeTrajectoryIndex));
      CommonOps_DDRM.scale(1.0 - interpolationFraction, PToReturn);
      CommonOps_DDRM.addEquals(PToReturn, interpolationFraction, PTrajectory.get(reverseTimeTrajectoryIndex - 1));

      return PToReturn;
   }

}
