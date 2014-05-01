package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import sun.misc.Unsafe;
import us.ihmc.utilities.exeptions.NoConvergenceException;
// javah -cp ../classes/:../../ThirdParty/ThirdPartyJars/EJML/EJML.jar -o ActiveSetQPMomentumOptimizer.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ActiveSetQPMomentumOptimizer 

public class ActiveSetQPMomentumOptimizer implements MomentumOptimizerInterface, Serializable
{
   public static final long serialVersionUID = -3703185211823067947L;
   static final int nPointsPerPlane = 4;
   static final int nSupportVectorsPerPoint=4;
   static final int nPlanes=4;
   static final int nWrench=6;
   static final int nRho = nPlanes*nPointsPerPlane*nSupportVectorsPerPoint;
   final int nDoF;
   
   boolean saveNoConvergeProblem=true;


   public DenseMatrix64F
      A, b, C,    //quad(vd)
     Js, ps, Ws,  //quad(vd)
     Jp, pp,  //quad(vd)
     
     WRho, Lambda, //reg

     WRhoSmoother, //quad(rho)

     rhoPrevMean, WRhoCoPPenalty, //quad(rho)

     QRho, c, rhoMin;  //constraints
     
   public DenseMatrix64F vd, rho;
   public DenseMatrix64F prevVd, prevRho;
   double optVal;
   //PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator;
   
   
   public ActiveSetQPMomentumOptimizer()
   {
      nDoF=-1;
   }
   
   public ActiveSetQPMomentumOptimizer(int _nDoF)
   {
         nDoF = _nDoF;
         vd = new DenseMatrix64F(nDoF,1);
         rho = new DenseMatrix64F(getRhoSize(),1);
         prevVd = new DenseMatrix64F(nDoF,1);
         prevRho = new DenseMatrix64F(getRhoSize(),1);

         Js = new DenseMatrix64F(nDoF,nDoF);
         ps = new DenseMatrix64F(nDoF,1);
         Jp = new DenseMatrix64F(nDoF, nDoF);
         pp = new DenseMatrix64F(nDoF, 1);
         Ws = new DenseMatrix64F(nDoF, nDoF);
         
         loadNativeLibrary();
         initializeNative(nDoF);
   }

   public void setSaveNoConvergeProblem(boolean saveNoConvergeProblem)
   {
      this.saveNoConvergeProblem = saveNoConvergeProblem;
   }
   
   public void dumpProblem(FileOutputStream of) throws IOException
   {
      ObjectOutputStream oos = new ObjectOutputStream(of);
      oos.writeObject(this);
      oos.close();
   }


   @Override
   public void reset()
   {
      Jp.zero();
      pp.zero();
      Js.zero();
      ps.zero();
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
   public void setInputs(DenseMatrix64F _a, DenseMatrix64F _b, DenseMatrix64F _momentumDotWeight, DenseMatrix64F _jSecondary, DenseMatrix64F _pSecondary,
         DenseMatrix64F _weightMatrixSecondary, DenseMatrix64F _WRho, DenseMatrix64F _Lambda, DenseMatrix64F _WRhoSmoother, DenseMatrix64F _rhoPrevAvg,
         DenseMatrix64F _WRhoCop, DenseMatrix64F _QRho, DenseMatrix64F _c, DenseMatrix64F _rhoMin)
   {
      A=_a;
      b=_b;
      C=_momentumDotWeight;
      if (!(A.numRows == nWrench && A.numCols == nDoF && b.numRows == nWrench && b.numCols == 1 && C.numRows == nWrench && C.numCols == nWrench))
         throw new RuntimeException("Incorrect input size A/b/c");

      //Constraint size may change, so copy
      CommonOps.insert(_jSecondary, Js, 0, 0);
      CommonOps.insert(_pSecondary, ps, 0, 0);
      CommonOps.insert(_weightMatrixSecondary, Ws, 0, 0);
      
      Jp.set(0, 0, Double.NaN); //mark as not used
      pp.set(0, 0, Double.NaN);

      WRho=_WRho;
      Lambda=_Lambda;
      
      WRhoSmoother =_WRhoSmoother;

      rhoPrevMean = _rhoPrevAvg;
      WRhoCoPPenalty = _WRhoCop;

      QRho = _QRho; 
      c = _c;
      rhoMin = _rhoMin;

   }

   @Override
//   public void setInputs(DenseMatrix64F a, DenseMatrix64F b, PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator,
//         DenseMatrix64F wrenchEquationRightHandSide, DenseMatrix64F momentumDotWeight, DenseMatrix64F dampedLeastSquaresFactorMatrix, DenseMatrix64F jPrimary,
//         DenseMatrix64F pPrimary, DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary)
   public void setInputs(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F momentumDotWeight, DenseMatrix64F jPrimary, DenseMatrix64F pPrimary,
         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary, DenseMatrix64F WRho, DenseMatrix64F Lambda,
         DenseMatrix64F WRhoSmoother, DenseMatrix64F rhoPrevAvg, DenseMatrix64F WRhoCop, DenseMatrix64F QRho, DenseMatrix64F c, DenseMatrix64F rhoMin)
   {
      setInputs(a, b, momentumDotWeight, jSecondary, pSecondary,
         weightMatrixSecondary, WRho, Lambda, WRhoSmoother, rhoPrevAvg,
         WRhoCop, QRho, c, rhoMin);
      CommonOps.insert(jPrimary, Jp, 0, 0);
      CommonOps.insert(pPrimary, pp, 0, 0);
   }
   
   public native void initializeNative(int nDoF);
   public native void resetActiveSet();
   
   public native int solveNative(
         double[] A, double[] b, double[] C, 
         double[] Jp, double[] pp,
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

      CommonOps.insert(rho, prevRho, 0, 0);
      CommonOps.insert(vd, prevVd, 0, 0);
      int iter = solveNative(
            A.getData(), b.getData(), C.getData(), 
            Jp.getData(),pp.getData(),
            Js.getData(), ps.getData(), Ws.getData(), 
            WRho.getData(), Lambda.getData(), 
            WRhoSmoother.getData(), 
            rhoPrevMean.getData(), WRhoCoPPenalty.getData(),
            QRho.getData(), c.getData(), rhoMin.getData(), 
            vd.getData(), rho.getData()  // in/out
      );
      if(iter <0)
      {
         if(saveNoConvergeProblem)
         {
                 String fileName = getClass().getSimpleName()+"_diverence"+System.nanoTime();
                 try{
                         FileOutputStream os = new FileOutputStream(fileName);
                         dumpProblem(os);
                         os.close();
                         System.out.println("NoConvergence log saved to "+ fileName);
                 }
                 catch(IOException e)
                 {
                    System.err.println("Attempted to write a log to "+fileName+" but failed..");
                 }
         }

         throw new NoConvergenceException();
      }else
         return iter;
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
   
   
   static String[] nativeLibraryCandidates = {"ActiveSetQPMomentumOptimizer_rel","ActiveSetQPMomentumOptimizer_msz","ActiveSetQPMomentumOptimizer"};
   static void loadNativeLibrary()
   {
      for(int i=0;i<nativeLibraryCandidates.length;i++)
      {
              try
              {
                      System.loadLibrary(nativeLibraryCandidates[i]);
                      break;
              }
              catch(UnsatisfiedLinkError e)
              {
                 if(i==(nativeLibraryCandidates.length-1))
                       throw(e);
              }
      }
   }

   
   public static void main(String[] arg) throws IOException, ClassNotFoundException, NoConvergenceException, IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException
   {
      FileInputStream is = new FileInputStream("../ValkyrieHardwareDrivers/ActiveSetQPMomentumOptimizer_diverence1398789891417766000");
      ObjectInputStream ois = new ObjectInputStream(is);
      ActiveSetQPMomentumOptimizer solver = (ActiveSetQPMomentumOptimizer) ois.readObject();
      ois.close();
      is.close();

      solver.setSaveNoConvergeProblem(false);
      solver.initializeNative(solver.nDoF);
      solver.solve();

   }


      

}
