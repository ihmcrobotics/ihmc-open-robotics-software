package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.BlockDiagSquareMatrix;
import us.ihmc.utilities.exceptions.NoConvergenceException;


/**
 * see super class for variable explanation
 * @author tingfan
 *
 */
public class CQPMomentumBasedOptimizer extends QPMomentumOptimizer
{
   private final ConstrainedQPSolver CQPsolver;
   private boolean firstCall;
   final boolean useBoxContraints;
   final boolean useBlockDiagMatrixclass=true;

   DenseMatrix64F AtC, JstWs, Q, QBlk1, QBlk2, f, fBlk1, fBlk2, Aeq, negA, beq, Ain, bin, x0, IdentityMatrix;
   DenseMatrix64F lb, ub;
   public CQPMomentumBasedOptimizer(int nDoF, ConstrainedQPSolver CQPsolver)
   {
      super(nDoF);

      // pre-allocation matrices to avoid gc()
      AtC = new DenseMatrix64F(nDoF, nWrench);
      if(useBlockDiagMatrixclass)
      {
         Q = new BlockDiagSquareMatrix(nDoF, nRho);
      }
      else
      {
         Q = new DenseMatrix64F(nDoF + nRho, nDoF + nRho);
      }
      QBlk1 = new DenseMatrix64F(nDoF, nDoF);
      QBlk2 = new DenseMatrix64F(nRho, nRho);

      f = new DenseMatrix64F(nDoF + nRho, 1);
      fBlk1 = new DenseMatrix64F(nDoF, 1);
      fBlk2 = new DenseMatrix64F(nRho, 1);

      Aeq = new DenseMatrix64F(nWrench + nDoF, nDoF + nRho);
      negA = new DenseMatrix64F(nWrench, nDoF);
      beq = new DenseMatrix64F(nWrench + nDoF, 1);
      x0 = new DenseMatrix64F(nDoF + nRho, 1);
      IdentityMatrix = CommonOps.identity(nDoF + nRho, nDoF + nRho);

      useBoxContraints = CQPsolver.supportBoxConstraints();

      if (useBoxContraints)
      {
         Ain = new DenseMatrix64F(0, nDoF + nRho);
         bin = new DenseMatrix64F(0, 1);
         lb = new DenseMatrix64F(nRho + nDoF, 1);
         ub = new DenseMatrix64F(nRho + nDoF, 1);
      }
      else
      {
         Ain = new DenseMatrix64F(nRho, nDoF + nRho);
         bin = new DenseMatrix64F(nRho, 1);
         lb = null;
         ub = null;
      }

      firstCall = true;
      this.CQPsolver = CQPsolver;
   }


   @Override
   public int solve() throws NoConvergenceException
   {
      CommonOps.multTransA(A, C, AtC);
      if (JstWs == null)
         JstWs = new DenseMatrix64F(nDoF, Ws.numCols);
      CommonOps.multTransA(Js, Ws, JstWs);

      /*
       * Q = [A'CA + Js'Ws Js + Lambda     0                                 ]
       *     [0                            Wp+Wpsm+Wpcop + QfeetCoP'QfeetCoP ]
       */

      CommonOps.mult(AtC, A, QBlk1);
      CommonOps.multAdd(JstWs, Js, QBlk1);
      CommonOps.addEquals(QBlk1, Lambda);

      CommonOps.add(WRho, WRhoSmoother, QBlk2);
      CommonOps.addEquals(QBlk2, WRhoCoPPenalty);
      CommonOps.multAddTransA(QfeetCop, QfeetCop, QBlk2);

      
      if(useBlockDiagMatrixclass)
      {
         ((BlockDiagSquareMatrix)Q).setBlock(QBlk1, 0);
         ((BlockDiagSquareMatrix)Q).setBlock(QBlk2, 1);
      }
      else
      {
              CommonOps.insert(QBlk1, Q, 0, 0);
              CommonOps.insert(QBlk2, Q, nDoF, nDoF);
      }
      
      CommonOps.add(1e-8, IdentityMatrix, Q, Q);    // regularization


      /*
       * f = -[A'C b   +  Js' Ws ps        ]
       *      [Wpsm Pprev   +   Wpcop Ppavg]
       */

      // blk1
      CommonOps.mult(AtC, b, fBlk1);
      CommonOps.multAdd(JstWs, ps, fBlk1);

      // add Js'Ws ps

      // blk2
      CommonOps.mult(WRhoSmoother, prevRho, fBlk2);
      CommonOps.multAdd(WRhoCoPPenalty, rhoPrevMean, fBlk2);

      // assemble
      CommonOps.insert(fBlk1, f, 0, 0);
      CommonOps.insert(fBlk2, f, nDoF, 0);
      CommonOps.changeSign(f);

      /*
       *        nDoF|nRho
       * Aeq = [-A   QRho] nWrench
       *       [Jp   Zero] nDoF
       *
       * beq = [c    pp  ]
       */
      boolean isPrimaryConstraintUsed = (Jp.numRows > 0) &&!Double.isNaN(Jp.get(0, 0));
      if (isPrimaryConstraintUsed)
      {
         Aeq.reshape(nWrench + Jp.numRows, nDoF + nRho);
         beq.reshape(nWrench + pp.numRows, 1);
         CommonOps.insert(Jp, Aeq, nWrench, 0);
         CommonOps.insert(pp, beq, nWrench, 0);
      }
      else
      {
         Aeq.reshape(nWrench, nDoF + nRho);
         beq.reshape(nWrench, 1);
      }

      CommonOps.scale(-1, A, negA);

      CommonOps.insert(negA, Aeq, 0, 0);
      CommonOps.insert(QRho, Aeq, 0, nDoF);
      CommonOps.insert(c, beq, 0, 0);


      /*
       * Equality Constraint Ain - nDoF nRho ---------------- | Zero| -I | nRho
       * ----------------
       */
      if (useBoxContraints)
      {
         CommonOps.fill(lb, Double.NEGATIVE_INFINITY);
         CommonOps.fill(ub, Double.POSITIVE_INFINITY);
         CommonOps.insert(rhoMin, lb, nDoF, 0);
      }
      else
      {
         for (int i = 0; i < nRho; i++)
         {
            Ain.set(i, i + nDoF, -1.0);
         }

         CommonOps.scale(-1, rhoMin, bin);
      }

      /*
       * initial x
       */
      CommonOps.insert(vd, x0, 0, 0);
      CommonOps.insert(rho, x0, nDoF, 0);

      int iter;
      if (useBoxContraints)
      {
         iter=CQPsolver.solve(Q, f, Aeq, beq, Ain, bin, lb, ub, x0, firstCall);
      }
      else
      {
         try
         {
            iter=CQPsolver.solve(Q, f, Aeq, beq, Ain, bin, x0, firstCall);
         }
         catch (Exception e)
         {
            System.out.println(Aeq);
            System.out.println(e.getMessage());

            throw e;
         }
      }

      firstCall = false;
      CommonOps.extract(x0, 0, nDoF, 0, 1, vd, 0, 0);
      CommonOps.extract(x0, nDoF, nDoF + nRho, 0, 1, rho, 0, 0);

      return iter; 
   }

}
