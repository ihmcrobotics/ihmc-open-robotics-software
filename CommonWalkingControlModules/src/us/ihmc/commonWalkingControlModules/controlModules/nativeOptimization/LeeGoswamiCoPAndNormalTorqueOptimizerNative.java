package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class LeeGoswamiCoPAndNormalTorqueOptimizerNative
{
   public final static int MAX_NUMBER_OF_CONTACTS = 2;
   private final static int vectorLength = 3;

   private static native void initialize();

   private static native ByteBuffer getPsiKBuffer();

   private static native ByteBuffer getKappaKBuffer();

   private static native ByteBuffer getEtaMinBuffer();

   private static native ByteBuffer getEtaMaxBuffer();

   private static native ByteBuffer getEtaDBuffer();

   private static native ByteBuffer getEpsilonBuffer();

   private static native ByteBuffer getEtaBuffer();

   private static native int solveNative();

   private static native double getOptValNative();

   private static final Object solveConch = new Object();

   private static DoubleBuffer psiKBuffer;
   private static DoubleBuffer kappaKBuffer;
   private static DoubleBuffer etaMinBuffer;
   private static DoubleBuffer etaMaxBuffer;
   private static DoubleBuffer etaDBuffer;
   private static DoubleBuffer epsilonBuffer;

   private static DoubleBuffer etaBuffer;

   public static DoubleBuffer setupBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());
      return buffer.asDoubleBuffer();
   }

   public static void setBufferToArray(DoubleBuffer buffer, double[] array)
   {
      buffer.rewind();
      buffer.put(array);
   }

   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.commonWalkingControlModules.lib", "LeeGoswamiCoPAndNormalTorqueOptimizer");

      initialize();

      psiKBuffer = setupBuffer(getPsiKBuffer());
      kappaKBuffer = setupBuffer(getKappaKBuffer());
      etaMinBuffer = setupBuffer(getEtaMinBuffer());
      etaMaxBuffer = setupBuffer(getEtaMaxBuffer());
      etaDBuffer = setupBuffer(getEtaDBuffer());
      epsilonBuffer = setupBuffer(getEpsilonBuffer());

      etaBuffer = setupBuffer(getEtaBuffer().asReadOnlyBuffer());

   }

   private final double[] eta = new double[MAX_NUMBER_OF_CONTACTS * vectorLength];
   double optval = 0.0;

   public LeeGoswamiCoPAndNormalTorqueOptimizerNative(double vectorLength)
   {
      assert LeeGoswamiCoPAndNormalTorqueOptimizerNative.vectorLength == vectorLength;
   }

   public int solve(double[] psiK, double[] kappaK, double[] etaMin, double[] etaMax, double[] etaD, double[] epsilon) throws NoConvergenceException
   {
      int numberOfIterations;

      synchronized (solveConch)
      {
         setBufferToArray(psiKBuffer, psiK);
         setBufferToArray(kappaKBuffer, kappaK);
         setBufferToArray(etaMinBuffer, etaMin);
         setBufferToArray(etaMaxBuffer, etaMax);
         setBufferToArray(etaDBuffer, etaD);
         setBufferToArray(epsilonBuffer, epsilon);

         numberOfIterations = solveNative();

         etaBuffer.rewind();
         etaBuffer.get(eta);

         optval = getOptValNative();

      }

      if (numberOfIterations < 0)
      {
         throw new NoConvergenceException();
      }

      return numberOfIterations;
   }
   
   public double[] getEta()
   {
      return eta;
   }

   public double getOptval()
   {
      return optval;
   }
   
   public static void main(String[] args)
   {
      
      double[] psiK = new double[vectorLength * MAX_NUMBER_OF_CONTACTS * vectorLength];
      double[] kappaK = new double[vectorLength];
      double[] etaMin = new double[MAX_NUMBER_OF_CONTACTS * vectorLength]; 
      double[] etaMax = new double[MAX_NUMBER_OF_CONTACTS * vectorLength];
      double[] etaD = new double[MAX_NUMBER_OF_CONTACTS * vectorLength]; 
      double[] epsilon = new double[MAX_NUMBER_OF_CONTACTS * vectorLength];

      load_default_data(psiK, kappaK, etaD, epsilon, etaMin, etaMax);

      LeeGoswamiCoPAndNormalTorqueOptimizerNative leeGoswamiCoPAndNormalTorqueOptimizerNative = new LeeGoswamiCoPAndNormalTorqueOptimizerNative(vectorLength);

      long time = System.nanoTime();
      for (int i = 0; true; i++)
      {
         if (i % 10000 == 0)
         {
            System.out.println("10000 iterations took " + (System.nanoTime() - time) / 1e9 + " seconds");
//            System.out.println(Arrays.toString(leeGoswamiCoPAndNormalTorqueOptimizerNative.getEta()));
            time = System.nanoTime();
         }
         
         try
         {
            leeGoswamiCoPAndNormalTorqueOptimizerNative.solve(psiK, kappaK, etaMin, etaMax, etaD, epsilon);
         }
         catch (NoConvergenceException e)
         {
         }
      }
   }

   private static void load_default_data(double[] Psi_k, double[] kappa_k, double[] eta_d, double[] epsilon, double[] etamin, double[] etamax) {
      Psi_k[0] = 0.20319161029830202;
      Psi_k[1] = 0.8325912904724193;
      Psi_k[2] = -0.8363810443482227;
      Psi_k[3] = 0.04331042079065206;
      Psi_k[4] = 1.5717878173906188;
      Psi_k[5] = 1.5851723557337523;
      Psi_k[6] = -1.497658758144655;
      Psi_k[7] = -1.171028487447253;
      Psi_k[8] = -1.7941311867966805;
      Psi_k[9] = -0.23676062539745413;
      Psi_k[10] = -1.8804951564857322;
      Psi_k[11] = -0.17266710242115568;
      Psi_k[12] = 0.596576190459043;
      Psi_k[13] = -0.8860508694080989;
      Psi_k[14] = 0.7050196079205251;
      Psi_k[15] = 0.3634512696654033;
      Psi_k[16] = -1.9040724704913385;
      Psi_k[17] = 0.23541635196352795;
      kappa_k[0] = -0.9629902123701384;
      kappa_k[1] = -0.3395952119597214;
      kappa_k[2] = -0.865899672914725;
      eta_d[0] = 0.7725516732519853;
      eta_d[1] = -0.23818512931704205;
      eta_d[2] = -1.372529046100147;
      eta_d[3] = 0.17859607212737894;
      eta_d[4] = 1.1212590580454682;
      eta_d[5] = -0.774545870495281;
      epsilon[0] = 1.2219578839321814;
      epsilon[1] = 1.3879712575556487;
      epsilon[2] = 1.9363836498604305;
      epsilon[3] = 1.9759954224729337;
      epsilon[4] = 1.6723836759128137;
      epsilon[5] = 1.9028341085383982;
      etamin[0] = 1.383003485172717;
      etamin[1] = -0.48802383468444344;
      etamin[2] = -1.631131964513103;
      etamin[3] = 0.6136436100941447;
      etamin[4] = 0.2313630495538037;
      etamin[5] = -0.5537409477496875;
      etamax[0] = -1.0997819806406723;
      etamax[1] = -0.3739203344950055;
      etamax[2] = -0.12423900520332376;
      etamax[3] = -0.923057686995755;
      etamax[4] = -0.8328289030982696;
      etamax[5] = -0.16925440270808823;
    }
   
}


