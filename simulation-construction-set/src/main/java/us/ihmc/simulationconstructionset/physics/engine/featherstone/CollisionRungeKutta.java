package us.ihmc.simulationconstructionset.physics.engine.featherstone;

public class CollisionRungeKutta implements java.io.Serializable
{
   private static final long serialVersionUID = 8085611439415747376L;

   private static final boolean REPORT = false;    // false; //true;

   double minStepSize_;
   double stepSize_;
   double accuracy_;
   double currentStepSize_;
   boolean verbose;
   boolean adaptive_;

   private int NVARS;

   public CollisionRungeKutta(int NVARS)
   {
      this.NVARS = NVARS;

      // Create memory objects once here:
      dydx = new double[NVARS];
      yend = new double[NVARS];
      yerr = new double[NVARS];

      x = new double[1];
      hnext = new double[1];
      hdid = new double[1];
      yscal = new double[NVARS];
      y = new double[NVARS];

      // dydx = new double[NVARS];

      // yerr = new double[NVARS];
      ytemp = new double[NVARS];


      ak2 = new double[NVARS];
      ak3 = new double[NVARS];
      ak4 = new double[NVARS];
      ak5 = new double[NVARS];
      ak6 = new double[NVARS];
      ytemp2 = new double[NVARS];

      currentStepSize_ = stepSize_ = 0.05;
      minStepSize_ = stepSize_ / 100;
      accuracy_ = 1.0e-6;
      adaptive_ = true;
   }

   public void setStepSize(double stepSize)
   {
      this.stepSize_ = stepSize;
      if (stepSize < 100 * minStepSize_)
         minStepSize_ = stepSize_ / 100;
   }

   public void setMinimumStepSize(double stepSize)
   {
      this.minStepSize_ = stepSize;
   }

   public void setAdaptive()
   {
      adaptive_ = true;
   }

   public void setNonAdaptive()
   {
      adaptive_ = false;
   }

   public void setAccuracy(double accuracy)
   {
      this.accuracy_ = accuracy;
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   public void setVerbose()
   {
      verbose = !verbose;
   }

   int[] nok = new int[1];
   int[] nbad = new int[1];

   public int goodSteps()
   {
      return nok[0];
   }

   public int badSteps()
   {
      return nbad[0];
   }

   public int steps()
   {
      return nok[0] + nbad[0];
   }

   public double stepSize()
   {
      return currentStepSize_;
   }

   public void integrate(double[] y, double[] range, CollisionDerivativeVector dv) throws ODEException, CollisionDerivativeException
   {
      // double start = range[0], end = range[1];

      double h = stepSize_;
      double hmin = minStepSize_;
      nok[0] = nbad[0] = 0;

      if (adaptive_)
      {
         odeint(y, range, accuracy_, h, hmin, nok, nbad, dv);

         if (verbose)
         {
            System.out.println("nok = " + nok[0] + "\tnbad = " + nbad[0]);
         }
      }
      else
      {
         rkdumb(y, range, h, dv);    // rkdumb(y, y.length, range, h, dv);
      }
   }

   double[] dydx;
   double[] yend;
   double[] yerr;

   void rkdumb(double[] ystart, double[] range, double h, CollisionDerivativeVector dv) throws CollisionDerivativeException
   {
      double start = range[0], end = range[1];

      int nSteps = (int) Math.abs((end - start) / h);
      if (nSteps < 1)
         nSteps = 1;
      h = (end - start) / nSteps;

      /*
       * double[] dydx = new double[NVARS];
       * double[] yend = new double[NVARS];
       * double[] yerr = new double[NVARS];
       */
      for (int step = 0; step < nSteps; step++)
      {
         double x = start + step * h;
         dv.derivs(x, ystart, dydx);
         rkck(ystart, dydx, x, h, yend, yerr, dv);

         if (REPORT)
            System.out.print("rkdumb: " + x + " ");    // +++JEP

         for (int n = 0; n < NVARS; n++)
         {
            ystart[n] = yend[n];
            if (REPORT)
               System.out.print(yend[n] + " ");    // +++JEP
         }

         if (REPORT)
            System.out.println();    // +++JEP
      }

      range[1] = start + (nSteps - 1) * h;    // +++JEP record the ending time...
   }

   static final int MAXSTP = 10000;
   static final double TINY = 1.0e-30;

   int kmax;
   int kount;
   double[] xp;
   double[][] yp;
   double dxsav;

   double[] x;
   double[] hnext;
   double[] hdid;
   double[] yscal;
   double[] y;

   // double[] dydx;  // Already defined for rkdumb

   void odeint(double[] ystart, double[] range, double eps, double h1, double hmin, int[] nok, int[] nbad, CollisionDerivativeVector dv)
           throws ODEException, CollisionDerivativeException
   {
      double x1 = range[0], x2 = range[1];

      /*
       * double[] x = new double[1];
       * double[] hnext = new double[1];
       * double[] hdid = new double[1];
       * double[] yscal = new double[NVARS];
       * double[] y = new double[NVARS];
       * double[] dydx = new double[NVARS];
       */

      x[0] = x1;
      double h = Math.abs(h1);
      if (x2 < x1)
         h = -h;
      nok[0] = nbad[0] = kount = 0;

      for (int i = 0; i < NVARS; i++)
      {
         y[i] = ystart[i];
      }

      double xsav = 0;
      if (kmax > 0)
         xsav = x[0] - dxsav * 2.0;

      if (dv.isStuck(y))
      {
         range[1] = x[0];

         return;
      }    // +++JEP return if stuck at the get go...

      for (int nstp = 1; nstp <= MAXSTP; nstp++)
      {
         dv.derivs(x[0], y, dydx);

         for (int i = 0; i < NVARS; i++)
         {
            yscal[i] = Math.abs(y[i]) + Math.abs(dydx[i] * h) + TINY;
         }

         if ((kmax > 0) && (kount < kmax - 1) && (Math.abs(x[0] - xsav) > Math.abs(dxsav)))
         {
            xp[++kount] = x[0];

            for (int i = 0; i < NVARS; i++)
            {
               yp[i][kount] = y[i];
            }

            xsav = x[0];
         }

         if ((x[0] + h - x2) * (x[0] + h - x1) > 0.0)
            h = x2 - x[0];
         rkqs(y, dydx, x, h, eps, yscal, hdid, hnext, dv);
         if (hdid[0] == h)
            ++nok[0];
         else
            ++nbad[0];

         if (((x[0] - x2) * (x2 - x1) >= 0.0) || (dv.isStuck(y)))    // +++JEP stop integrating if stuck!
         {
            for (int i = 0; i < NVARS; i++)
            {
               ystart[i] = y[i];
            }

            if (kmax != 0)
            {
               xp[++kount] = x[0];

               for (int i = 0; i < NVARS; i++)
               {
                  yp[i][kount] = y[i];
               }
            }

            range[1] = x[0];    // +++JEP record the ending time...

            return;
         }



         if (Math.abs(hnext[0]) <= hmin)
         {
            throw new ODEException("Step size too small in odeint.");

            // error("Step size too small in odeint");  //+++JEP Turn this error message off for now...
            // System.exit(0);
         }

         h = hnext[0];

         // System.out.println("h: " + h);
         currentStepSize_ = h;    // added for comphys
      }

      System.err.println("Too many steps in routine odeint.");
//      throw new ODEException("Too many steps in routine odeint.");

      // error("Too many steps in routine odeint.  NVARS: " + NVARS);
   }

   static final double SAFETY = 0.9;
   static final double PGROW = -0.2;
   static final double PSHRNK = -0.25;
   static final double ERRCON = 1.89e-4;    // ERRCON = 1.89e-4; +++JEP.  This seems way too low!!! This is error as percent of accuracy!

   // double[] yerr;
   double[] ytemp;

   void rkqs(double[] y, double[] dydx, double[] x, double htry, double eps, double[] yscal, double[] hdid, double[] hnext, CollisionDerivativeVector dv)
           throws ODEException, CollisionDerivativeException
   {
      double errmax = 0;

      /*
       * double[] yerr = new double[NVARS];
       * double[] ytemp = new double[NVARS];
       */
      double h = htry;
      for (;;)
      {
         rkck(y, dydx, x[0], h, ytemp, yerr, dv);
         errmax = 0;

         for (int i = 0; i < NVARS; i++)
         {
            errmax = Math.max(errmax, Math.abs(yerr[i] / yscal[i]));
         }

         errmax /= eps;
         if (errmax <= 1.0)
            break;
         double htemp = SAFETY * h * Math.pow(errmax, PSHRNK);
         h = ((h >= 0.0) ? Math.max(htemp, 0.1 * h) : Math.min(htemp, 0.1 * h));
         double xnew = x[0] + h;
         if (xnew == x[0])
         {
            throw new ODEException("stepsize underflow in rkqs.");

            // error("stepsize underflow in rkqs");
         }
      }

      if (errmax > ERRCON)
      {
         hnext[0] = SAFETY * h * Math.pow(errmax, PGROW);

         // / +++JEP.  Just don't let it go too low of step size?

         /*
          * System.out.println("SAFETY: " + SAFETY);
          * System.out.println("h: " + h);
          * System.out.println("errmax: " + errmax);
          * System.out.println("PGROW: " + PGROW);
          *
          * System.out.println("hnext: " + hnext[0]);
          */


      }
      else
         hnext[0] = 5.0 * h;


      x[0] += (hdid[0] = h);

      if (REPORT)
         System.out.print("rkqs: " + x[0] + " ");    // +++JEP

      for (int i = 0; i < NVARS; i++)
      {
         y[i] = ytemp[i];
         if (REPORT)
            System.out.print(ytemp[i] + " ");    // +++JEP
      }

      if (REPORT)
         System.out.println();    // +++JEP
   }

   static final double a2 = 0.2, a3 = 0.3, a4 = 0.6, a5 = 1.0, a6 = 0.875, b21 = 0.2, b31 = 3.0 / 40.0, b32 = 9.0 / 40.0, b41 = 0.3, b42 = -0.9, b43 = 1.2,
                       b51 = -11.0 / 54.0, b52 = 2.5, b53 = -70.0 / 27.0, b54 = 35.0 / 27.0, b61 = 1631.0 / 55296.0, b62 = 175.0 / 512.0, b63 = 575.0 / 13824.0,
                       b64 = 44275.0 / 110592.0, b65 = 253.0 / 4096.0, c1 = 37.0 / 378.0, c3 = 250.0 / 621.0, c4 = 125.0 / 594.0, c6 = 512.0 / 1771.0,
                       dc5 = -277.0 / 14336.0;
   static final double dc1 = c1 - 2825.0 / 27648.0, dc3 = c3 - 18575.0 / 48384.0, dc4 = c4 - 13525.0 / 55296.0, dc6 = c6 - 0.25;


   double[] ak2;
   double[] ak3;
   double[] ak4;
   double[] ak5;
   double[] ak6;
   double[] ytemp2;

   void rkck(double[] y, double[] dydx, double x, double h, double[] yout, double[] yerr, CollisionDerivativeVector dv) throws CollisionDerivativeException
   {
      /*
       * double[] ak2 = new double[NVARS];
       * double[] ak3 = new double[NVARS];
       * double[] ak4 = new double[NVARS];
       * double[] ak5 = new double[NVARS];
       * double[] ak6 = new double[NVARS];
       * double[] ytemp2 = new double[NVARS];
       */
      for (int i = 0; i < NVARS; i++)
      {
         ytemp2[i] = y[i] + b21 * h * dydx[i];
      }

      dv.derivs(x + a2 * h, ytemp2, ak2);

      for (int i = 0; i < NVARS; i++)
      {
         ytemp2[i] = y[i] + h * (b31 * dydx[i] + b32 * ak2[i]);
      }

      dv.derivs(x + a3 * h, ytemp2, ak3);

      for (int i = 0; i < NVARS; i++)
      {
         ytemp2[i] = y[i] + h * (b41 * dydx[i] + b42 * ak2[i] + b43 * ak3[i]);
      }

      dv.derivs(x + a4 * h, ytemp2, ak4);

      for (int i = 0; i < NVARS; i++)
      {
         ytemp2[i] = y[i] + h * (b51 * dydx[i] + b52 * ak2[i] + b53 * ak3[i] + b54 * ak4[i]);
      }

      dv.derivs(x + a5 * h, ytemp2, ak5);

      for (int i = 0; i < NVARS; i++)
      {
         ytemp2[i] = y[i] + h * (b61 * dydx[i] + b62 * ak2[i] + b63 * ak3[i] + b64 * ak4[i] + b65 * ak5[i]);
      }

      dv.derivs(x + a6 * h, ytemp2, ak6);

      for (int i = 0; i < NVARS; i++)
      {
         yout[i] = y[i] + h * (c1 * dydx[i] + c3 * ak3[i] + c4 * ak4[i] + c6 * ak6[i]);
      }

      for (int i = 0; i < NVARS; i++)
      {
         yerr[i] = h * (dc1 * dydx[i] + dc3 * ak3[i] + dc4 * ak4[i] + dc5 * ak5[i] + dc6 * ak6[i]);
      }
   }

   void error(String msg)
   {
      // System.err.println("comphys.numerics.RungeKutta: " + msg);
      System.out.println("comphys.numerics.RungeKutta: " + msg);
   }

   class ODEException extends Exception implements java.io.Serializable
   {
      private static final long serialVersionUID = -9101473542052720197L;

      public ODEException()
      {
      }

      public ODEException(String message)
      {
         super(message);
      }
   }

}
