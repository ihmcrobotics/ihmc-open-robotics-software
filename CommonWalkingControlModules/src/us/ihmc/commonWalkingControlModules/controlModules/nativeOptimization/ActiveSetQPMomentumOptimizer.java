package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.utilities.exeptions.NoConvergenceException;
// javah -cp ../classes/:../../ThirdParty/ThirdPartyJars/EJML/EJML.jar -o ActiveSetQPMomentumOptimizer.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ActiveSetQPMomentumOptimizer 

public class ActiveSetQPMomentumOptimizer implements MomentumOptimizerInterface
{
   static{
      System.loadLibrary("ActiveSetQPMomentumOptimizer"); 
   }
   static final int nPointsPerPlane = 4;
   static final int nSupportVectorsPerPoint=4;
   static final int nPlanes=4;
   static final int nWrench=6;
   static final int nRho = nPlanes*nPointsPerPlane*nSupportVectorsPerPoint;
   final int nDoF;
   
   DenseMatrix64F
      A, b, C,    //quad(vd)
     Js, ps, Ws,  //quad(vd)
     
     WRho, Lambda, //reg

     WRhoSmoother, //quad(rho)

     rhoPrevMean, WRhoCoPPenalty, //quad(rho)

     QRho, c, rhoMin;  //constraints
     
   DenseMatrix64F vd, rho;
   double optVal;
   //PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator;
   
   public ActiveSetQPMomentumOptimizer(int _nDoF)
   {
         nDoF = _nDoF;
         vd = new DenseMatrix64F(nDoF,1);
         rho = new DenseMatrix64F(getRhoSize(),1);

         Js = new DenseMatrix64F(nDoF,nDoF);
         ps = new DenseMatrix64F(nDoF,1);
         Ws = new DenseMatrix64F(nDoF, nDoF);
         initializeNative(nDoF);
   }


   @Override
   public void reset()
   {
      Arrays.fill(rho.getData(), 0);
      Arrays.fill(vd.getData(), 0);
   }


   @Override
   public int getRhoSize()
   {
      return nPlanes*nPointsPerPlane*nSupportVectorsPerPoint;
   }

   @Override
   public int getNSupportVectors()
   {
      return nSupportVectorsPerPoint;
   }

   @Override
   public int getNPointsPerPlane()
   {
      return nPointsPerPlane;
   }

   @Override
   public void setRateOfChangeOfGroundReactionForceRegularization(DenseMatrix64F _wRhoSmoother)
   {
      WRhoSmoother = _wRhoSmoother;
   }


   @Override
   public int getNPlanes()
   {
      return nPlanes;
   }

   @Override
   public void setInputs(DenseMatrix64F _a, DenseMatrix64F _b, PlaneContactWrenchMatrixCalculator _wrenchMatrixCalculator,
         DenseMatrix64F _wrenchEquationRightHandSide, DenseMatrix64F _momentumDotWeight, DenseMatrix64F _dampedLeastSquaresFactorMatrix,
         DenseMatrix64F _jSecondary, DenseMatrix64F _pSecondary, DenseMatrix64F _weightMatrixSecondary)
   {
      A=_a;
      b=_b;
      C=_momentumDotWeight;
      if (!(A.numRows == nWrench && A.numCols == nDoF && b.numRows == nWrench && b.numCols == 1 && C.numRows == nWrench && C.numCols == nWrench))
         throw new RuntimeException("Incorrect input size A/b/c");

      CommonOps.insert(_jSecondary, Js, 0, 0);
      CommonOps.insert(_pSecondary, ps, 0, 0);
      CommonOps.insert(_weightMatrixSecondary, Ws, 0, 0);
//      Js.insuu=_jSecondary;
//      ps=_pSecondary;
//      Ws=_weightMatrixSecondary;
      /*
      if(!( Js.numRows == nDoF && Js.numCols == nDoF && ps.numRows == nDoF && ps.numCols == 1 && Ws.numRows == nDoF && Ws.numCols == nDoF) ) 
      {
         //throw new RuntimeException("Incorrect input size Js/ps/Ws");
         
      }*/

      WRho=_wrenchMatrixCalculator.getWRho();
      Lambda=_dampedLeastSquaresFactorMatrix;
      
      WRhoSmoother = _wrenchMatrixCalculator.getWRhoSmoother();

      rhoPrevMean = _wrenchMatrixCalculator.getRhoPreviousAverage();
      WRhoCoPPenalty = _wrenchMatrixCalculator.getWRhoPenalizer();

      QRho = _wrenchMatrixCalculator.getQRho();       
      c = _wrenchEquationRightHandSide;
      rhoMin = _wrenchMatrixCalculator.getRhoMin();

   }
   
   public native void initializeNative(int nDoF);
   public native void resetActiveSet();
   
   public native int solveNative(
         double[] A, double[] b, double[] C, 
         double[] Js, double[] ps, double[] Ws,
         double[] WRho, double[] Lambda,
         double[] WRhoSmoother, 
         double[] rhoPrevMean, double[] WRhoCoPPenalty,
         double[] QRho, double[] c, double[] rhoMin,
         double[] vd, double[] rho);
   
   /*
    * min(qdd,rho) 0.5 qdd' A qdd + b'qdd 
    * 
    * minimize quad(A * vd - b, C) + quad(Js * vd - ps, Ws) + quad(rho, WRho) + quad(vd, Lambda) + quad(rho - rhoPrevious, WRhoSmoother) + quad(rho - rhoPreviousMean, WRhoCoPPenalty)
    *    subject to
    *      Qrho * rho == A * vd + c # simple force balance
    *      rho >= rhoMin
    *    end
    */
   @Override
   public int solve() throws NoConvergenceException
   {

      return solveNative(
            A.getData(), b.getData(), C.getData(), 
            Js.getData(), ps.getData(), Ws.getData(), 
            WRho.getData(), Lambda.getData(), 
            WRhoSmoother.getData(), 
            rhoPrevMean.getData(), WRhoCoPPenalty.getData(),
            QRho.getData(), c.getData(), rhoMin.getData(), 
            vd.getData(), rho.getData()  // in/out
      );
   }

   @Override
   public DenseMatrix64F getOutputRho()
   {
      return rho;
   }
   
   
   @Override
   public DenseMatrix64F getOutputJointAccelerations()
   {
      return vd;
   }

   @Override
   public double getOutputOptVal()
   {
      return optVal;
   }

}
