package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;
import java.util.Arrays;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class SimpleHQPNative
{
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.commonWalkingControlModules.lib", "SimpleHQPNative");
   }

   public static final int nParams = 14;

   public int nStages;
   public int ASize;
   public int bSize;
   public int AInit;
   public int CSize;
   public int dSize;
   public int CInit;
   public int ESize;
   public int fSize;
   public int EInit;
   public int GSize;
   public int hSize;
   public int GInit;
   public int xSize;

   private double[] x;
   public double[] Params;

   private DoubleBuffer ADoubleBuffer;
   private DoubleBuffer bDoubleBuffer;
   private DoubleBuffer bTypeDoubleBuffer;
   private DoubleBuffer CDoubleBuffer;
   private DoubleBuffer dDoubleBuffer;
   private DoubleBuffer dTypeDoubleBuffer;
   private DoubleBuffer EDoubleBuffer;
   private DoubleBuffer fDoubleBuffer;
   private DoubleBuffer fTypeDoubleBuffer;
   private DoubleBuffer GDoubleBuffer;
   private DoubleBuffer hDoubleBuffer;
   private DoubleBuffer hTypeDoubleBuffer;
   private DoubleBuffer xDoubleBuffer;
   private DoubleBuffer ParamsDoubleBuffer;

   public void initializeJava(int nStages, int ASize, int bSize, int AInit, int CSize, int dSize, int CInit, int ESize, int fSize, int EInit, int GSize,
                              int hSize, int GInit, int xSize)
   {
      this.nStages = nStages;
      this.ASize = ASize;
      this.bSize = bSize;
      this.AInit = AInit;
      this.CSize = CSize;
      this.dSize = dSize;
      this.CInit = CInit;
      this.ESize = ESize;
      this.fSize = fSize;
      this.EInit = EInit;
      this.GSize = GSize;
      this.hSize = hSize;
      this.GInit = GInit;
      this.xSize = xSize;

      x = new double[xSize];

      Params = new double[nParams];
      Params[0] = this.nStages;
      Params[1] = this.ASize;
      Params[2] = this.bSize;
      Params[3] = this.AInit;
      Params[4] = this.CSize;
      Params[5] = this.dSize;
      Params[6] = this.CInit;
      Params[7] = this.ESize;
      Params[8] = this.fSize;
      Params[9] = this.EInit;
      Params[10] = this.GSize;
      Params[11] = this.hSize;
      Params[12] = this.GInit;
      Params[13] = this.xSize;
      initParams();

      ParamsDoubleBuffer = setupBuffer(getParamsBuffer());
      setBufferToArray(ParamsDoubleBuffer, Params);

      initialize();

      ADoubleBuffer = setupBuffer(getABuffer());
      bDoubleBuffer = setupBuffer(getbBuffer());
      bTypeDoubleBuffer = setupBuffer(getbTypeBuffer());
      CDoubleBuffer = setupBuffer(getCBuffer());
      dDoubleBuffer = setupBuffer(getdBuffer());
      dTypeDoubleBuffer = setupBuffer(getdTypeBuffer());
      EDoubleBuffer = setupBuffer(getEBuffer());
      fDoubleBuffer = setupBuffer(getfBuffer());
      fTypeDoubleBuffer = setupBuffer(getfTypeBuffer());
      GDoubleBuffer = setupBuffer(getGBuffer());
      hDoubleBuffer = setupBuffer(gethBuffer());
      hTypeDoubleBuffer = setupBuffer(gethTypeBuffer());
      xDoubleBuffer = setupBuffer(getxBuffer());
   }

   private static native ByteBuffer getABuffer();

   private static native ByteBuffer getbBuffer();

   private static native ByteBuffer getbTypeBuffer();

   private static native ByteBuffer getCBuffer();

   private static native ByteBuffer getdBuffer();

   private static native ByteBuffer getdTypeBuffer();

   private static native ByteBuffer getEBuffer();

   private static native ByteBuffer getfBuffer();

   private static native ByteBuffer getfTypeBuffer();

   private static native ByteBuffer getGBuffer();

   private static native ByteBuffer gethBuffer();

   private static native ByteBuffer gethTypeBuffer();

   private static native ByteBuffer getxBuffer();

   private static native ByteBuffer getParamsBuffer();

   private static native void solveInCPP();

   private static native void initParams();

   private static native void initialize();

   private static native void clearMemory();

   private static final Object solveConch = new Object();

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

   private void solve(double[] A, double[] b, double[] bType, double[] C, double[] d, double[] dType, double[] E, double[] f, double[] fType, double[] G,
                      double[] h, double[] hType)
   {
      synchronized (solveConch)
      {
         setBufferToArray(ADoubleBuffer, A);
         setBufferToArray(bDoubleBuffer, b);
         setBufferToArray(bTypeDoubleBuffer, bType);
         setBufferToArray(CDoubleBuffer, C);
         setBufferToArray(dDoubleBuffer, d);
         setBufferToArray(dTypeDoubleBuffer, dType);
         setBufferToArray(EDoubleBuffer, E);
         setBufferToArray(fDoubleBuffer, f);
         setBufferToArray(fTypeDoubleBuffer, fType);
         setBufferToArray(GDoubleBuffer, G);
         setBufferToArray(hDoubleBuffer, h);
         setBufferToArray(hTypeDoubleBuffer, hType);

         solveInCPP();

         xDoubleBuffer.rewind();
         xDoubleBuffer.get(x);
      }
   }

   public static void main(String[] args)
   {
      SimpleHQPNative SimpleHQPNative = new SimpleHQPNative();

      int stages = 3;
      int sizeA = 3;
      int sizeb = 2;
      int initA = 0;
      int sizeC = 1;
      int sized = 1;
      int initC = 1;
      int sizeE = 1;
      int sizef = 1;
      int initE = 2;
      int sizeG = 0;
      int sizeh = 0;
      int initG = 0;
      int sizex = 3;

      SimpleHQPNative.initializeJava(stages, sizeA, sizeb, initA, sizeC, sized, initC, sizeE, sizef, initE, sizeG, sizeh, initG, sizex);

      double[] A = new double[SimpleHQPNative.bSize * SimpleHQPNative.ASize];
      double[] b = new double[SimpleHQPNative.bSize];
      double[] bType = new double[SimpleHQPNative.bSize];
      double[] C = new double[SimpleHQPNative.dSize * SimpleHQPNative.CSize];
      double[] d = new double[SimpleHQPNative.dSize];
      double[] dType = new double[SimpleHQPNative.dSize];
      double[] E = new double[SimpleHQPNative.fSize * SimpleHQPNative.ESize];
      double[] f = new double[SimpleHQPNative.fSize];
      double[] fType = new double[SimpleHQPNative.fSize];
      double[] G = new double[SimpleHQPNative.hSize * SimpleHQPNative.GSize];
      double[] h = new double[SimpleHQPNative.hSize];
      double[] hType = new double[SimpleHQPNative.hSize];

      A[0] = 1;
      A[1] = 0;
      A[2] = 0;
      A[3] = 0;
      A[4] = 0;
      A[5] = 1;
      b[0] = 1;
      b[1] = 2;
      bType[1] = 2;
      C[0] = 1;
      d[0] = 3;
      E[0] = 1;
      f[0] = 5;

      long start = System.nanoTime();
      int rep = 10;
      for (int i = 0; i < rep; i++)
      {
         SimpleHQPNative.solve(A, b, bType, C, d, dType, E, f, fType, G, h, hType);;
      }

      long end = System.nanoTime();
      System.out.println("------");
      System.out.println(Arrays.toString(SimpleHQPNative.x));
      System.out.println("------");
      System.out.println("time :" + ((end - start) / 1e6) / rep + " ms");

      clearMemory();


   }
}
