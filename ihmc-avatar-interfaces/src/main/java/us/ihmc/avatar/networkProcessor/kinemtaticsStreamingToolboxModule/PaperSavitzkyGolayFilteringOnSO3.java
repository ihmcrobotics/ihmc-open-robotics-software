package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Java implementation of <a href="https://github.com/MaartenJongeneel/Paper-Savitzky-Golay-filtering-on-SO3/blob/main/SavitzkyGolaySO3.m"> Original GitHub
 * Repository</a>
 */
public class PaperSavitzkyGolayFilteringOnSO3
{
   // User inputs
   private double Fc = 1;            // Signal frequency                  [Hz]
   private double a = 2;             // Signal amplitude                  [deg]
   private double te = 2;            // Signal length                     [s]
   private double Fs = 1000;         // Sampling frequency fine grid      [Hz]
   private int m = 5;                // Down-sampling rate                [-]
   private double sigma = 0.06;      // Standard deviation of added noise [rad]
   private int n = 20;               // Window size SG-filter             [-]
   private int p = 3;                // Savitzky Golay filter order       [-]

   // Computed values
   private double dt1 = 1.0 / Fs;                      // Time step                         [s]
   private double dt2 = m / Fs;                        // Time step lower sampled           [s]
   private double[] t1 = newDoubleArray(0, dt1, te);   // Signal time vector                [s]
   private double[] t2 = newDoubleArray(0, dt2, te);   // Signal time vector lower sampled  [s]
   private int N1 = t1.length;                      // Number of samples                 [-]
   private int N2 = t2.length;                      // Number of samples lower sampled   [-]

   // Preallocate memory
   private List<Vector3D> omg = IntStream.range(0, N1).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Vector3D> omg_FD = IntStream.range(0, N2).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Vector3D> domg = IntStream.range(0, N1).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Vector3D> domg_FD = IntStream.range(0, N2).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Matrix3D> R = IntStream.range(0, N1).mapToObj(i -> new Matrix3D()).collect(Collectors.toList());
   private List<Matrix3D> R_noise = IntStream.range(0, N2).mapToObj(i -> new Matrix3D()).collect(Collectors.toList());

   private List<Vector3D> phi = IntStream.range(0, N1).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Vector3D> dphi = IntStream.range(0, N1).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Vector3D> ddphi = IntStream.range(0, N1).mapToObj(i -> new Vector3D()).collect(Collectors.toList());
   private List<Vector3D> g_noise = IntStream.range(0, N2).mapToObj(i -> new Vector3D()).collect(Collectors.toList());

   // Creating data on SO(3)
   //Create a random sine wave in R3 with first and second order derivative
   // lambda0 = randn(3,1);
   // lambda1 = randn(3,1);

   // Vectors below are created by randn(3,1) but placed here s.t. we can give
   //the values in the paper and show the corresponding plots
   Vector3D lambda0 = new Vector3D(-0.4831, 0.6064, -2.6360);
   Vector3D lambda1 = new Vector3D(0.9792, 1.4699, -0.4283);

   private static double[] newDoubleArray(double start, double step, double end)
   {
      int length = (int) ((end - start) / step) + 1;
      double[] array = new double[length];
      for (int i = 0; i < length; i++)
      {
         array[i] = start + i * step;
      }
      return array;
   }

   private static int[] newIntArray(int start, int step, int end)
   {
      int length = (int) ((end - start) / step);
      int[] array = new int[length];
      for (int i = 0; i < length; i++)
      {
         array[i] = start + i * step;
      }
      return array;
   }

   public PaperSavitzkyGolayFilteringOnSO3()
   {
      for (int ii = 0; ii < N1; ii++)
      {
         double freq = 2 * Math.PI * Fc;
         phi.get(ii).scaleAdd(a * Math.sin(freq * t1[ii]), lambda1, lambda0);
         dphi.get(ii).setAndScale(a * freq * Math.cos(freq * t1[ii]), lambda1);
         ddphi.get(ii).setAndScale(-a * freq * freq * Math.sin(freq * t1[ii]), lambda1);

         //Compute analytically the rotation matrices, ang. vel., and ang. acc.
         // R(:,:,ii) = expSO3(phi(:,ii));
         R.get(ii).set(expSO3(phi.get(ii)));
         // omg(:,ii) = dexpSO3(phi(:,ii))*dphi(:,ii);
         dexpSO3(phi.get(ii)).transform(dphi.get(ii), omg.get(ii));
         // domg(:,ii) = DdexpSO3(phi(:,ii),dphi(:,ii))*dphi(:,ii) +  dexpSO3(phi(:,ii))*ddphi(:,ii);
         Vector3D temp = new Vector3D();
         DdexpSO3(phi.get(ii), dphi.get(ii)).transform(dphi.get(ii), temp);
         dexpSO3(phi.get(ii)).transform(ddphi.get(ii), domg.get(ii));
         domg.get(ii).add(temp);
      }

      // Noisy, lower sampled signal ("measurement")
      int cnt = 0;
      Random random = new Random(456);
      for (int ii = 0; ii < N1; ii += m)
      {
         // R_noise(:,:,cnt) = expSO3(phi(:,ii)+sigma*randn(3,1));
         Vector3D noise = EuclidCoreRandomTools.nextVector3D(random);
         noise.scale(sigma);
         Matrix3DTools.multiply(expSO3(noise), R.get(ii), R_noise.get(cnt));
         cnt++;
      }

      // ---------------- Applying the Savitzky-Golay filter ----------------- //
      // Now, from the noisy lower sampled data, we want to get back the estimated
      // rotation matrix, angular velocity and angular acceleration
      sgolayfiltSO3 result = new sgolayfiltSO3(R_noise, p, n, 1 / dt2);

      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2(SimulationConstructionSet2.doNothingPhysicsEngine());
      scs2.setDT(dt1);

      YoQuaternion rawOrientation = new YoQuaternion("rawOrientation", scs2.getRootRegistry());
      YoQuaternion noisyOrientation = new YoQuaternion("noisyOrientation", scs2.getRootRegistry());
      YoQuaternion filteredOrientation = new YoQuaternion("filteredOrientation", scs2.getRootRegistry());

      YoVector3D rawAngularVelocity = new YoVector3D("rawAngularVelocity", scs2.getRootRegistry());
      YoVector3D noisyAngularVelocity = new YoVector3D("noisyAngularVelocity", scs2.getRootRegistry());
      YoVector3D filteredAngularVelocity = new YoVector3D("filteredAngularVelocity", scs2.getRootRegistry());

      YoVector3D rawAngularAcceleration = new YoVector3D("rawAngularAcceleration", scs2.getRootRegistry());
      YoVector3D noisyAngularAcceleration = new YoVector3D("noisyAngularAcceleration", scs2.getRootRegistry());
      YoVector3D filteredAngularAcceleration = new YoVector3D("filteredAngularAcceleration", scs2.getRootRegistry());
      YoVector3D fdAngularAcceleration = new YoVector3D("fdAngularAcceleration", scs2.getRootRegistry());

      for (int i = 0; i < N1; i++)
      {
         rawOrientation.set(new RotationMatrix(R.get(i)));
         //         noisyOrientation.set(new RotationMatrix(R_noise.get(i)));
         //         filteredOrientation.set(new RotationMatrix(result.R_est.get(i)));

         rawAngularVelocity.set(omg.get(i));
         //         noisyAngularVelocity.set(omg_FD.get(i));
         //         filteredAngularVelocity.set(result.omg_est.get(i));

         rawAngularAcceleration.set(domg.get(i));
         //         noisyAngularAcceleration.set(domg_FD.get(i));
         //         filteredAngularAcceleration.set(result.domg_est.get(i));

         if (i > 0)
         {
            fdAngularAcceleration.sub(omg.get(i), omg.get(i - 1));
            fdAngularAcceleration.scale(1 / dt1);
         }

         scs2.simulateNow(1);
      }

      scs2.start(true, false, false);
   }

   public static Matrix3D expSO3(Vector3DReadOnly a)
   {
      double phi = a.norm();
      Matrix3D res = new Matrix3D();
      res.setIdentity();
      if (phi != 0)
      {
         Matrix3D aHat = hat(a);
         aHat.scale(Math.sin(phi) / phi);
         res.add(aHat);
         aHat = hat(a);
         aHat.multiply(aHat);
         aHat.scale((1 - Math.cos(phi)) / (phi * phi));
         res.add(aHat);
      }
      return res;
   }

   public static Matrix3D hat(Vector3DReadOnly a)
   {
      Matrix3D res = new Matrix3D();
      res.set(0, -a.getZ(), a.getY(), a.getZ(), 0, -a.getX(), -a.getY(), a.getX(), 0);
      return res;
   }

   public static Matrix3D dexpSO3(Vector3DReadOnly a)
   {
      double phi = a.norm();

      Matrix3D res = new Matrix3D();
      res.setIdentity();

      if (phi != 0)
      {
         Matrix3D aHat = hat(a);
         double beta = MathTools.square(Math.sin(phi / 2.0)) / (MathTools.square(phi / 2.0));
         double alpha = Math.sin(phi) / phi;

         aHat.scale(0.5 * beta);
         res.add(aHat);
         aHat = hat(a);
         aHat.multiply(aHat);
         aHat.scale((1 - alpha) / MathTools.square(phi));
         res.add(aHat);
      }

      return res;
   }

   public static Matrix3D DdexpSO3(Vector3DReadOnly x, Vector3DReadOnly z)
   {
      double phi = x.norm();
      Matrix3D hatx = hat(x);
      Matrix3D hatz = hat(z);

      Matrix3D hatxhatz = new Matrix3D();
      hatxhatz.set(hatx);
      hatxhatz.multiply(hatz);
      Matrix3D hatzhatx = new Matrix3D();
      hatzhatx.set(hatz);
      hatzhatx.multiply(hatx);

      double beta = MathTools.square(Math.sin(phi / 2.0)) / (MathTools.square(phi / 2.0));
      double alpha = Math.sin(phi) / phi;
      double phi2 = MathTools.square(phi);

      Matrix3D res = new Matrix3D();
      res.set(hatz);
      res.scale(0.5 * beta);

      Matrix3D temp = new Matrix3D();
      // 1/phi^2*(1-alpha)*(hatx*hatz+hatz*hatx)
      temp.set(hatxhatz);
      temp.add(hatzhatx);
      temp.scale((1.0 - alpha) / phi2);
      res.add(temp);

      // 1/phi^2*(alpha-beta)*(x'*z)*hatx
      temp.set(outer(x, z));
      temp.multiply(hatx);
      temp.scale((alpha - beta) / phi2);
      res.add(temp);

      // 1/phi^2*(beta/2-3/phi^2*(1-alpha))*(x'*z)*hatx*hatx
      temp.set(outer(x, z));
      temp.multiply(hatx);
      temp.multiply(hatx);
      temp.scale((beta / 2.0 - 3.0 / phi2 * (1.0 - alpha)) / phi2);
      res.add(temp);
      return res;
   }

   public static Matrix3D outer(Vector3DReadOnly a, Vector3DReadOnly b)
   {
      Matrix3D res = new Matrix3D();
      res.set(a.getX() * b.getX(),
              a.getX() * b.getY(),
              a.getX() * b.getZ(),
              a.getY() * b.getX(),
              a.getY() * b.getY(),
              a.getY() * b.getZ(),
              a.getZ() * b.getX(),
              a.getZ() * b.getY(),
              a.getZ() * b.getZ());
      return res;
   }

   public static Vector3D vee(Matrix3D mat)
   {
      double xi1 = (mat.getM21() - mat.getM12()) / 2;
      double xi2 = (mat.getM02() - mat.getM20()) / 2;
      double xi3 = (mat.getM10() - mat.getM01()) / 2;
      return new Vector3D(xi1, xi2, xi3);
   }

   public static Vector3D logm(Matrix3D m)
   {
      Vector3D result = new Vector3D();
      RotationVectorConversion.convertMatrixToRotationVector(m.getM00(),
                                                             m.getM01(),
                                                             m.getM02(),
                                                             m.getM10(),
                                                             m.getM11(),
                                                             m.getM12(),
                                                             m.getM20(),
                                                             m.getM21(),
                                                             m.getM22(),
                                                             result);
      return result;
   }

   public static void main(String[] args)
   {
      PaperSavitzkyGolayFilteringOnSO3 result = new PaperSavitzkyGolayFilteringOnSO3();
   }

   public static class sgolayfiltSO3
   {
      private final List<Matrix3D> R_est = new ArrayList<>();
      private final List<Vector3D> omg_est = new ArrayList<>();
      private final List<Vector3D> domg_est = new ArrayList<>();
      private final TDoubleArrayList tf = new TDoubleArrayList();

      public sgolayfiltSO3(List<Matrix3D> R, int p, int n, double freq)
      {
         // Computed values
         int N = R.size();         // Number of samples in the sequence    [-]
         double dt = 1 / freq;              // Time step lower sampled              [s]
         double te = N * dt;                // Total lenght of the sequence         [s]
         double[] ts = newDoubleArray(0, dt, te);           // Signal time vector                   [s]
         int[] w = newIntArray(-n, 1, n);                 // Window for Golay                     [-]
         Matrix3D I = new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1);               // Short hand notation                  [-]
         for (int i = n; i < N - (n + 1); i++)
         { // tf = ts((n+1):(N-(n+1)));
            tf.add(ts[i]); // Time vector filtered signal          [s]
         }

         // Savitzky-Golay
         // For each time step (where we can apply the window)

         for (int ii = n; ii < N - (n + 1); ii++)
         {
            // Build matrix A and vector b based on window size w
            DMatrixRMaj A = new DMatrixRMaj(3 * w.length, 3 * (p + 1));
            DMatrixRMaj b = new DMatrixRMaj(3 * w.length, 1);
            for (int jj = 0; jj < w.length; jj++)
            {
               // Time difference between 0^th element and w(jj)^th element
               double Dt = (ts[ii + w[jj]] - ts[ii]);
               // Determine row of A matrix
               I.get(3 * jj, 0, A);
               for (int kk = 1; kk <= p; kk++)
               {
                  Matrix3D Ajj_kk = new Matrix3D(I);
                  Ajj_kk.scale(1.0 / kk * Math.pow(Dt, kk));
                  Ajj_kk.get(3 * jj, 3 * kk, A);
               }

               Matrix3D R_diff = new Matrix3D();
               R_diff.set(R.get(ii + w[jj]));
               R_diff.multiplyInvertOther(R.get(ii));
               Vector3D b_jj = logm(R_diff);
               b_jj.get(3 * jj, 0, b);
            }
            // Solve the LS problem
            DMatrixRMaj Apinv = new DMatrixRMaj(3 * (p + 1), 3 * w.length);
            CommonOps_DDRM.pinv(A, Apinv);
            DMatrixRMaj rho = new DMatrixRMaj(3 * (p + 1), 1);
            CommonOps_DDRM.mult(Apinv, b, rho);

            // Obtain the coefficients of rho
            Vector3D rho0 = new Vector3D(rho.get(0), rho.get(1), rho.get(2));
            Vector3D rho1 = new Vector3D(rho.get(3), rho.get(4), rho.get(5));
            Vector3D rho2 = new Vector3D(rho.get(6), rho.get(7), rho.get(8));

            // Compute analytically the rotation matrices, ang. vel., and ang. acc.
            Matrix3D R_est_ii = new Matrix3D();
            R_est_ii.set(expSO3(rho0));
            R_est_ii.multiply(R.get(ii));
            R_est.add(R_est_ii);
            Vector3D omg_est_ii = new Vector3D();
            dexpSO3(rho0).transform(rho1, omg_est_ii);
            omg_est.add(omg_est_ii);

            Vector3D domg_est_ii = new Vector3D();
            DdexpSO3(rho0, rho1).transform(rho1, domg_est_ii);
            Vector3D temp = new Vector3D();
            dexpSO3(rho0).transform(rho2, temp);
            domg_est_ii.add(temp);
            domg_est.add(domg_est_ii);
         }
      }
   }
}
