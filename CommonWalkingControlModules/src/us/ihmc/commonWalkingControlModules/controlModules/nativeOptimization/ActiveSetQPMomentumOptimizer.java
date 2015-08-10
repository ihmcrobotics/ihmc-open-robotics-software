package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.Arrays;

import org.ejml.ops.CommonOps;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import com.sun.jna.Library;
import com.sun.jna.Native;

// javah -cp ../classes/:../../ThirdParty/ThirdPartyJars/EJML/EJML.jar -o ActiveSetQPMomentumOptimizer.h us.ihmc.commonWalkingControlModules.controlModules
// .nativeOptimization.ActiveSetQPMomentumOptimizer


public class ActiveSetQPMomentumOptimizer extends QPMomentumOptimizer implements MomentumOptimizerInterface, Serializable
{
   public static final long serialVersionUID = -3703185211823067947L;
   public static final int MAX_ITER = -1;
   
   boolean saveNoConvergeProblem=true;


   public int[] activeSet;
   final boolean useJNA;
   
   
   public ActiveSetQPMomentumOptimizer(int _nDoF, boolean _useJNA)
   {
         super(_nDoF);
         useJNA = _useJNA;

         loadNativeLibraries();
         initializeNative(nDoF, MAX_ITER);
         
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
   
   private native void initializeNative(int nDoF, int maxIter);
   public void initlaize(int nDoF, int maxIter)
   {
      if(useJNA)
         JnaInterface.INSTANCE.initializeNative(nDoF, maxIter);
      else
         initializeNative(nDoF, maxIter);
   }
   
   
   
   @Override
   public int solve() throws NoConvergenceException
   {
      return solve(false);
   }
   
   /*
    * read ActiveSetQPMomentumOptimizer.cpp for parameter comments
    */
   public int solve(boolean clearActiveSet) throws NoConvergenceException
   {
      if(activeSet==null || activeSet.length != nRho) //#inequality constraints
      {
         System.err.println(this.getClass().getSimpleName() + ": warning - invalid activeset, regenerating");
         activeSet = new int[nRho]; 
      }

      CommonOps.insert(rho, prevRho, 0, 0);
      CommonOps.insert(vd, prevVd, 0, 0);
      if(activeSet==null)
         throw new NullPointerException("activeSet not initialize properly");
      if (clearActiveSet)
         Arrays.fill(activeSet, 0);
      
      iter=0;
      if(useJNA)
      {
         iter = JnaInterface.INSTANCE.solveNative(
            A.getData(), b.getData(), C.getData(), 
            Jp.getData(),pp.getData(),
            Js.getData(), ps.getData(), Ws.getData(), 
            WRho.getData(), Lambda.getData(), 
            WRhoSmoother.getData(), 
            rhoPrevMean.getData(), WRhoCoPPenalty.getData(),
            QRho.getData(), c.getData(), rhoMin.getData(), 
            vd.getData(), rho.getData(),  // in/out
            activeSet
        );
      }
      else
      {
         iter = solveNative(
            A.getData(), b.getData(), C.getData(), 
            Jp.getData(),pp.getData(),
            Js.getData(), ps.getData(), Ws.getData(), 
            WRho.getData(), Lambda.getData(), 
            WRhoSmoother.getData(), 
            rhoPrevMean.getData(), WRhoCoPPenalty.getData(),
            QRho.getData(), c.getData(), rhoMin.getData(), 
            vd.getData(), rho.getData(),  // in/out
            activeSet
        );
      }
      if(iter <0)
      {
         boolean tryAgain=false;
         if(tryAgain)
         {
            iter = solveNative(
                A.getData(), b.getData(), C.getData(), 
                Jp.getData(),pp.getData(),
                Js.getData(), ps.getData(), Ws.getData(), 
                WRho.getData(), Lambda.getData(), 
                WRhoSmoother.getData(), 
                rhoPrevMean.getData(), WRhoCoPPenalty.getData(),
                QRho.getData(), c.getData(), rhoMin.getData(), 
                vd.getData(), rho.getData(),  // in/out
                activeSet
            );
          System.out.println("Try again, iter=" + iter +"\n -----");
         }
         for(int i=0;i<vd.data.length;i++)
            if(Double.isNaN(vd.get(i)))
               break;
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

         throw new NoConvergenceException(iter);
      }else
         return iter;
   }

   private native int solveNative(
         double[] A, double[] b, double[] C, 
         double[] Jp, double[] pp,
         double[] Js, double[] ps, double[] Ws,
         double[] WRho, double[] Lambda,
         double[] WRhoSmoother, 
         double[] rhoPrevMean, double[] WRhoCoPPenalty,
         double[] QRho, double[] c, double[] rhoMin,
         double[] vd, double[] rho,
         int[] activeSet);
   
   public interface JnaInterface extends Library
   {
      int solveNative(
            double[] A, double[] b, double[] C, 
            double[] Jp, double[] pp,
            double[] Js, double[] ps, double[] Ws,
            double[] WRho, double[] Lambda,
            double[] WRhoSmoother, 
            double[] rhoPrevMean, double[] WRhoCoPPenalty,
            double[] QRho, double[] c, double[] rhoMin,
            double[] vd, double[] rho,
            int[] activeSet);
      void initializeNative(int nDoF, int maxIter);
      
      String NATIVE_LIBRARY = NativeLibraryLoader.extractLibrary("us.ihmc.commonWalkingControlModules.lib", "ActiveSetQPMomentumOptimizer_rel");
      JnaInterface INSTANCE = (JnaInterface) Native.loadLibrary(NATIVE_LIBRARY, JnaInterface.class);
   }

   
   
   static String[] nativeLibraryCandidates = {"ActiveSetQPMomentumOptimizer_rel","ActiveSetQPMomentumOptimizer_msz","ActiveSetQPMomentumOptimizer"};
   /*
   static
   {
      loadNativeLibraries();
   }*/
   
   public static void loadNativeLibraries()
   {
      for(int i=0;i<nativeLibraryCandidates.length;i++)
      {
              try
              {
                    NativeLibraryLoader.loadLibrary("us.ihmc.commonWalkingControlModules.lib", nativeLibraryCandidates[i]);
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
      String fileName="../ValkyrieHardwareDrivers/ActiveSetQPMomentumOptimizer_diverence1402652193717240000";
      FileInputStream is = new FileInputStream(fileName);
      ObjectInputStream ois = new ObjectInputStream(is);
      ActiveSetQPMomentumOptimizer solver = (ActiveSetQPMomentumOptimizer) ois.readObject();
      ois.close();
      is.close();

      
      solver.setSaveNoConvergeProblem(false);
      solver.loadNativeLibraries();
      solver.initializeNative(solver.nDoF,MAX_ITER);
      solver.solve();
      System.err.flush();
      System.out.println("Reconstruct problem from ");
      System.out.println("iter="+solver.iter);
      System.out.println("Opt="+solver.optVal);
      System.out.println("ActiveSet="+Arrays.toString(solver.activeSet));
      System.out.println("iter="+solver.solve());
      System.out.println("ActiveSet="+Arrays.toString(solver.activeSet));
   }


      

}
