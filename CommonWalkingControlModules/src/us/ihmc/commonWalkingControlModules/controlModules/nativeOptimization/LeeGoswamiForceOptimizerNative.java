package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class LeeGoswamiForceOptimizerNative
{
   public static final int NUMBER_OF_SUPPORT_VECTORS = 4;
   public static final int MAX_NUMBER_OF_CONTACTS = 2;

   private final static int n = 3 * MAX_NUMBER_OF_CONTACTS;
   private final static int m = NUMBER_OF_SUPPORT_VECTORS * MAX_NUMBER_OF_CONTACTS;

   private static native void initialize();

   private static native ByteBuffer getPhiBuffer();

   private static native ByteBuffer getXiBuffer();

   private static native ByteBuffer getRhoBuffer();

   private static native int solveNative(double epsilonf);

   private static native double getOptValNative();

   private static final Object solveConch = new Object();

   private static final DoubleBuffer phiDoubleBuffer;
   private static final DoubleBuffer xiDoubleBuffer;
   private static final DoubleBuffer rhoDoubleBuffer;

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
      NativeLibraryLoader.loadLibrary("us.ihmc.commonWalkingControlModules.lib", "LeeGoswamiForceOptimizer");

      initialize();

      phiDoubleBuffer = setupBuffer(getPhiBuffer());
      xiDoubleBuffer = setupBuffer(getXiBuffer());
      rhoDoubleBuffer = setupBuffer(getRhoBuffer().asReadOnlyBuffer());
   }

   private final double[] rho = new double[m];
   private double optval = 0.0;

   public LeeGoswamiForceOptimizerNative(double nRows, double nCols)
   {
      assert nRows == n;
      assert nCols == m;

   }

   public int solve(double[] phi, double[] xi, double epsilonf) throws NoConvergenceException
   {
      int numberOfIterations;

      synchronized (solveConch)
      {
         setBufferToArray(phiDoubleBuffer, phi);
         setBufferToArray(xiDoubleBuffer, xi);
         
         numberOfIterations = solveNative(epsilonf);

         rhoDoubleBuffer.rewind();
         rhoDoubleBuffer.get(rho);

         optval = getOptValNative();

      }

      if (numberOfIterations < 0)
      {
         throw new NoConvergenceException();
      }

      return numberOfIterations;

   }

   public double[] getRho()
   {
      return rho;
   }

   public double getOptval()
   {
      return optval;
   }

   public static void main(String[] args) throws NoConvergenceException
   {
      double[] phi = new double[n * m];
      double[] xi = new double[n];
      double[] epsilon_f = new double[1];

      load_default_data(phi, xi, epsilon_f);

      LeeGoswamiForceOptimizerNative leeGoswamiForceOptimizerNative = new LeeGoswamiForceOptimizerNative(n, m);

      long time = System.nanoTime();
      for (int i = 0; true; i++)
      {
         if (i % 10000 == 0)
         {
            System.out.println("10000 iterations took " + (System.nanoTime() - time) / 1e9 + " seconds");
//            System.out.println(Arrays.toString(leeGoswamiForceOptimizerNative.getRho()));
            time = System.nanoTime();
         }
         leeGoswamiForceOptimizerNative.solve(phi, xi, epsilon_f[0]);
      }
   }

   private static void load_default_data(double[] Phi, double[] xi, double[] epsilon_f)
   {
      Phi[0] = 0.20319161029830202;
      Phi[1] = 0.8325912904724193;
      Phi[2] = -0.8363810443482227;
      Phi[3] = 0.04331042079065206;
      Phi[4] = 1.5717878173906188;
      Phi[5] = 1.5851723557337523;
      Phi[6] = -1.497658758144655;
      Phi[7] = -1.171028487447253;
      Phi[8] = -1.7941311867966805;
      Phi[9] = -0.23676062539745413;
      Phi[10] = -1.8804951564857322;
      Phi[11] = -0.17266710242115568;
      Phi[12] = 0.596576190459043;
      Phi[13] = -0.8860508694080989;
      Phi[14] = 0.7050196079205251;
      Phi[15] = 0.3634512696654033;
      Phi[16] = -1.9040724704913385;
      Phi[17] = 0.23541635196352795;
      Phi[18] = -0.9629902123701384;
      Phi[19] = -0.3395952119597214;
      Phi[20] = -0.865899672914725;
      Phi[21] = 0.7725516732519853;
      Phi[22] = -0.23818512931704205;
      Phi[23] = -1.372529046100147;
      Phi[24] = 0.17859607212737894;
      Phi[25] = 1.1212590580454682;
      Phi[26] = -0.774545870495281;
      Phi[27] = -1.1121684642712744;
      Phi[28] = -0.44811496977740495;
      Phi[29] = 1.7455345994417217;
      Phi[30] = 1.9039816898917352;
      Phi[31] = 0.6895347036512547;
      Phi[32] = 1.6113364341535923;
      Phi[33] = 1.383003485172717;
      Phi[34] = -0.48802383468444344;
      Phi[35] = -1.631131964513103;
      Phi[36] = 0.6136436100941447;
      Phi[37] = 0.2313630495538037;
      Phi[38] = -0.5537409477496875;
      Phi[39] = -1.0997819806406723;
      Phi[40] = -0.3739203344950055;
      Phi[41] = -0.12423900520332376;
      Phi[42] = -0.923057686995755;
      Phi[43] = -0.8328289030982696;
      Phi[44] = -0.16925440270808823;
      Phi[45] = 1.442135651787706;
      Phi[46] = 0.34501161787128565;
      Phi[47] = -0.8660485502711608;
      xi[0] = -0.8880899735055947;
      xi[1] = -0.1815116979122129;
      xi[2] = -1.17835862158005;
      xi[3] = -1.1944851558277074;
      xi[4] = 0.05614023926976763;
      xi[5] = -1.6510825248767813;
      epsilon_f[0] = 0.967171064703173;
   }
}
