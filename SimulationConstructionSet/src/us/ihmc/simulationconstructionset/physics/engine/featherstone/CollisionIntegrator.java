package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.numericalMethods.QuarticRootFinder;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.CollisionRungeKutta.ODEException;


public class CollisionIntegrator implements java.io.Serializable
{
   /*
    * private static final double U_STUCK_THRESH = 0.0001;
    * private static final double ACCURACY = 0.0002;
    * private static final double H_MIN = 0.0000000001, H_START = 0.001; //H_MIN = 0.00000001, H_START = 0.001; //H_MIN = 0.000001, H_START = 0.001;
    * private static final double UZ_OVERSHOOT = 0.00001;
    */

   private static final long serialVersionUID = -6083539629185172597L;
   private static final double U_STUCK_THRESH = 0.0001;    // U_STUCK_THRESH = 0.0001;
   private static final double ACCURACY = 1e-7;
   private static final double
      H_MIN = 1.0E-10, H_START = 0.001;    // H_MIN = 1.0E-10, H_START = 0.001; //H_MIN = 0.00000001, H_START = 0.001; //H_MIN = 0.000001, H_START = 0.001;
   private static final double UZ_OVERSHOOT = 0.0005;    // 0.001

   private static final double
      PZ_STEP_SIZE_MIN = 1.0E-7, PZ_STEP_SIZE_MAX = 10.0;

   private Matrix3D K, K_inv = new Matrix3D();

   // private Vector3d r1, r2, u1, u2;
   private double epsilon, mu;

   private boolean stableSticking;
   private boolean amStuck;    // , hadBadCompression;

   private Vector3D u0 = new Vector3D();
   Vector3D Kx = new Vector3D(), Ky = new Vector3D(), Kz = new Vector3D();

   public CollisionIntegrator()
   {
      collisionRungeKutta = new CollisionRungeKutta(3);

      /*
       * collisionRungeKutta.setAdaptive();
       * collisionRungeKutta.setStepSize(H_START);
       * collisionRungeKutta.setMinimumStepSize(H_MIN);
       * collisionRungeKutta.setVerbose(false);
       * collisionRungeKutta.setAccuracy(ACCURACY);
       */

      pzRungeKutta = new CollisionRungeKutta(4);

      /*
       * pzRungeKutta.setAdaptive();
       * pzRungeKutta.setStepSize(H_START);
       * pzRungeKutta.setMinimumStepSize(H_MIN);
       * pzRungeKutta.setVerbose(false);
       * pzRungeKutta.setAccuracy(ACCURACY);
       */
   }

   public static void main(String[] args)
   {
      System.out.println("Hello World");

      CollisionIntegrator integrator = new CollisionIntegrator();


      /*
       * Matrix3d K_test = new Matrix3d(8.0,-2.0,1.0,-2.0,3.0,-1.0,1.0,-1.0,5.0);
       * double mu = 0.2, epsilon = 0.2;
       * Vector3d u0_test = new Vector3d(-2.0, 1.0, -4.0);
       */


      /*
       * Matrix3d K_test = new Matrix3d(4.83333,0.0,0.0,0.0,4.833333,0.0,0.0,0.0,1.5);
       * double mu = 0.92, epsilon = 1.0;
       * Vector3d u0_test = new Vector3d(0.0, 0.0, -1.0);
       */

      /*
       * Matrix3d K_test = new Matrix3d(3.47985, 0.0100, 0.223515, 0.0100, 3.5, 0.0100, 0.223515, 0.0100, 1.020146);
       * double mu = 0.05, epsilon = 1.0;
       * Vector3d u0_test = new Vector3d(0.0, 0.0, -1.0);
       */

      /*
       * Matrix3d K_test = new Matrix3d(1.1342222504102195, 0.004126728884482201, -0.15691706096001176,
       *                             0.004126728884482199, 0.025381577079665128, 0.004414281746073675,
       *                             -0.15691706096001173, 0.004414281746073675, 0.06421269879067157);
       *
       * double mu = 0.7, epsilon = 0.1;
       * Vector3d u0_test = new Vector3d(-5.895158654917366E-4, 1.8749539576258544E-4, -0.0016297500787469943);
       */



      // p_coll: (0.17188135946929903, -0.007288630118566598, -0.24648651514990955)

      /*
       * Matrix3d K_test = new Matrix3d(1.1283380515752726, -0.001809908560091365, 0.17617678415118457,
       *                             -0.0018099085600913653, 0.025437857575848575, 0.004807094008384275,
       *                               0.17617678415118465, 0.0048070940083842755, 0.07008821141980301);
       *
       *
       * double mu = 0.7, epsilon = 0.1;
       * Vector3d u0_test = new Vector3d(5.806705594191019E-4, -7.915630494231113E-4, -0.011444529447608695);
       */

      /*
       * Matrix3d K_test = new Matrix3d(1.0884180123959373, -0.0016746159372512137, -0.273476754908481,
       *                             -0.001674615937251212, 0.025733497695040443, -0.0073012463637908784,
       *                             -0.27347675490848106, -0.007301246363790878, 0.12299541129420415);
       *
       *
       * double mu = 0.7, epsilon = 0.1;
       * Vector3d u0_test = new Vector3d(5.049818637140807E-4, -4.2306740568656334E-4, -0.0010165991967400496);
       */


      /*
       * Matrix3d K_test = new Matrix3d(1.2291934004316973, 0.00105078599490031, 0.13012964047319764,
       *                               0.001050785994900307, 0.43590647432237956, -0.20342283466057104,
       *                               0.13012964047319764, -0.2034228346605711, 0.12623720152295714);
       *
       * double mu = 0.7, epsilon = 0.1;
       * Vector3d u0_test = new Vector3d(-0.0895,-0.5198,-0.71915);
       */

      // Vector3d u0_test = new Vector3d(-0.792456072811943, -0.5812959495565716, -0.7866333086521968);


      /*
       * Matrix3d K_test = new Matrix3d(1.6450778034099955, 0.7982095897676131, -0.005188629210542688,
       *                               0.7982095897676127, 0.43458463722653984, -0.03705665798843,
       *                               -0.005188629210542542, -0.03705665798843, 0.15694363350221135);
       *
       * double mu = 0.7, epsilon = 0.1;
       * Vector3d u0_test = new Vector3d(-1.2873982706843987, -0.3749851697997516, -2.350018921463037);
       */


      Matrix3D K_test = new Matrix3D(1.9795607798930628, -0.23966928371230545, 0.2574790610887272, -0.23966928371230545, 0.06310349438764432,
                                     -0.012154180483161334, 0.2574790610887272, -0.012154180483161352, 0.19089180777144152);

      double mu = 0.7, epsilon = 3.0;    // .0;
      Vector3D u0_test = new Vector3D(-0.46351582426916077, 0.061475623099959374, -4.4499062468302507E-4);



      Matrix3D K_test_inv = new Matrix3D(K_test);
      K_test_inv.invert();
      System.out.println("K_test_inv: " + K_test_inv);


      integrator.setup(K_test, u0_test, epsilon, mu);

      // System.out.println("Beta for this K is :  " + integrator.solveBeta(K_test, mu));

      Vector3D final_output = new Vector3D();
      integrator.integrate(final_output);

      System.out.println("u0: " + u0_test);

      System.out.println("final ux: " + final_output.getX() + ", uy: " + final_output.getY() + ", uz: " + final_output.getZ());    // + ", Wz: " + final_output[3]);
      Vector3D delta_u = new Vector3D();
      delta_u.sub(final_output, u0_test);
      System.out.println("delta_u: " + delta_u);

      Vector3D impulse = new Vector3D(delta_u);
      K_test_inv.transform(impulse);
      System.out.println("Impulse:  " + impulse);


   }

   @SuppressWarnings("unused")
   private RotationMatrix K_pseudo = new RotationMatrix(), K_trans = new RotationMatrix();

   public void setup(Matrix3D K, Vector3D u0, double epsilon, double mu)
   {
      collisionRungeKutta.setAdaptive();
      collisionRungeKutta.setStepSize(H_START);
      collisionRungeKutta.setMinimumStepSize(H_MIN);
      collisionRungeKutta.setVerbose(false);
      collisionRungeKutta.setAccuracy(ACCURACY);

      pzRungeKutta.setAdaptive();
      pzRungeKutta.setStepSize(H_START);
      pzRungeKutta.setMinimumStepSize(H_MIN);
      pzRungeKutta.setVerbose(false);
      pzRungeKutta.setAccuracy(ACCURACY);


      this.K = K;
      this.u0 = u0;

      this.epsilon = epsilon;
      this.mu = mu;

      // System.out.println("K: " + K);

      K_inv.set(K);

      double inverseMinimum = 1e-11;
      if (Math.abs(K_inv.determinant()) < inverseMinimum)
      {
         System.err.println("Warning: K is not invertible in " + getClass().getSimpleName() + ". K_inv.determinant() = " + K_inv.determinant());
         System.err.println("K = " + K);
         // Cheesy Solution for now.  Look at the diagonal entries and if any are zero, assume that they are the null space and add a little to the other columns:
         // System.out.println("Having trouble inverting K.  Using cheesy inverse.");

         if (Math.abs(K_inv.getM00()) < inverseMinimum)
            K_inv.setM00(inverseMinimum);
         if (Math.abs(K_inv.getM11()) < inverseMinimum)
            K_inv.setM11(inverseMinimum);
         if (Math.abs(K_inv.getM22()) < inverseMinimum)
            K_inv.setM22(inverseMinimum);

         if (Math.abs(K_inv.determinant()) < inverseMinimum)
         {
            K_inv.setM00(K_inv.getM00() + inverseMinimum);
            K_inv.setM11(K_inv.getM11() + inverseMinimum);
            K_inv.setM22(K_inv.getM22() + inverseMinimum);
         }
      }

      K_inv.invert();

      // System.out.println("K_inv: " + K_inv);

      // Extract Kx, Ky, Kz:
      K.getRow(0, Kx);
      K.getRow(1, Ky);
      K.getRow(2, Kz);

      // System.out.println("Kx: " + Kx);
      // Compute if stable sticking or not.  Mirtich p. 65.

      if (K_inv.getElement(0, 2) * K_inv.getElement(0, 2) + K_inv.getElement(1, 2) * K_inv.getElement(1, 2)
            <= mu * mu * K_inv.getElement(2, 2) * K_inv.getElement(2, 2))
         stableSticking = true;
      else
         stableSticking = false;

      amStuck = false;

      // hadBadCompression = false;
   }

   private final Vector3D u_final = new Vector3D();
   private final Vector3D delta_u = new Vector3D();

   public void computeImpulse(Vector3D impulse)
   {
      // System.out.println("K: " + K);
      // System.out.println("u0: " + u0);
      // System.out.println("epsilon: " + epsilon);
      // System.out.println("mu: " + mu);

      integrate(u_final);

      // System.out.println("u_final: " + u_final);

      delta_u.set(u_final);
      delta_u.sub(this.u0);

      // System.out.println("delta_u: " + delta_u);

      impulse.set(delta_u);
      K_inv.transform(impulse);

      // System.out.println("impulse: " + impulse);


   }

   public void computeMicroImpulse(Vector3D impulse)
   {
      // Figure out impulse to stop u0 in its tracks and reverse it...

      impulse.set(u0);

      // impulse.scale(-2.0); //-1.5);
      impulse.scale(-2.1);    // +++JEP.  Use 2.1 instead of 2.0 to prevent major ground penetration...

      // impulse.x = impulse.x * (-1.0); //(-2.0); //-1.5);
      // impulse.y = impulse.y * (-1.0);
      // impulse.z = impulse.z * (-3.0);

      K_inv.transform(impulse);

   }


   private Vector3D zeta_B = new Vector3D();
   double[] pz_output = new double[4];
   double[] compression_output = new double[4];
   double[] restitution_output = new double[4];

   public void integrate(Vector3D u_fin)
   {
      double ux = u0.getX(), uy = u0.getY(), uz = u0.getZ(), Wz = 0.0;

      // Check to make sure that uz is initially increasing toward zero (Kz dot zeta(theta) > 0).  If not, then need to integrate with respect to pz first,
      // as in Mirtich p. 87.

      double utot = Math.sqrt(u0.getX() * u0.getX() + u0.getY() * u0.getY());
      zeta.set(-mu * u0.getX() / utot, -mu * u0.getY() / utot, 1.0);

      // System.out.println("Kz.dot(zeta): " + Kz.dot(zeta));

      int nn = 0;
      while (Kz.dot(zeta) < 0.0)
      {
         nn++;

//          System.out.println("Kz.dot(zeta)<0.0!!!!!!  Need to do integration with respect to pz first!!!!");

         // if (nn>20)System.out.println("Before pz integration (ux, uy, uz, Wz) = (" + ux + ", " + uy + ", " + uz + ", " + Wz + ")");

         double pz_step_size = (nn / 1000.0) * Math.abs(uz / K.getM22());    // 0.005 *
         if (pz_step_size > PZ_STEP_SIZE_MAX)
            pz_step_size = PZ_STEP_SIZE_MAX;
         if (pz_step_size < PZ_STEP_SIZE_MIN)
            pz_step_size = PZ_STEP_SIZE_MIN;

         integrateWRTpz(ux, uy, uz, Wz, pz_step_size, pz_output);
         ux = pz_output[0];
         uy = pz_output[1];
         uz = pz_output[2];
         Wz = pz_output[3];

         // System.out.println("After pz integration (ux, uy, uz, Wz) = (" + ux + ", " + uy + ", " + uz + ", " + Wz + ")");

         utot = Math.sqrt(ux * ux + uy * uy);
         zeta.set(-mu * ux / utot, -mu * uy / utot, 1.0);

         // if (Kz.dot(zeta) > 0.0) System.out.println("After pz integration (ux, uy, uz, Wz) = (" + ux + ", " + uy + ", " + uz + ", " + Wz + ")");

         // System.out.println("nn:  " + nn);

         /*
          * if ((Kz.dot(zeta) > 0.0) && (nn > 20))
          * {
          * System.out.println("nn:  " + nn);
          * System.out.print("K: "  + K);
          * System.out.println("u0: " + u0);
          * System.out.println("u_pz: (" + ux + ", " + uy + ", " + uz + ", " + Wz + ")   ");
          * System.out.println("amStuck: " + amStuck);
          * System.out.println();
          * }
          */

         // u_fin.set(0.0,0.0,0.0);
         // return;

         if (amStuck)
         {
            // System.out.println("Am stuck during pz integration");
            break;
         }
      }

      if (!amStuck)
      {
         if (uz < UZ_OVERSHOOT)    // Only do this if compressing.  Otherwise in restitution
         {
            try
            {
               integrateCompression(ux, uy, uz, Wz, compression_output);
               ux = compression_output[0];
               uy = compression_output[1];
               uz = compression_output[2];
               Wz = compression_output[3];
            }

            catch (CollisionDerivativeException e)
            {
               // System.out.println("Exception in integrateCompression: " + e);
               // +++JEP.  This happens when we should be integrating with respect to pz.  What should we do???
               if (!amStuck)
               {
                  // System.out.println("Had bad compression, yet am not stuck!!!! BAD!!!");
                  // System.out.println("u: (" + ux + ", " + uy + ", " + uz + ", " + Wz + ")   ");
                  // System.out.println("compression_output:  (" + compression_output[0] + ", " + compression_output[1] + ", " + compression_output[2] + ", " + compression_output[3] + ")");
               }

               // amStuck = true;  // +++JEP.  Might as well just throw away ux, uy and let uz bounce.
               // ux = 0.0; uy = 0.0;
               // Wz = 0.0;
               // uz = 0.0;
               uz = UZ_OVERSHOOT;    // 0.0; // Just stop it dead in it's tracks!!!
            }
         }
      }

      if (amStuck)
      {
         // If Wz > 0.0, then something went wrong here (Integrated too far).  If so, then set Wz = 0.0; +++JEP
         if (Wz > 0.0)
            Wz = 0.0;

         if (this.stableSticking)
         {
            // Stable Sticking during Compression.  Mirtich p. 74;
            // System.out.println("Stable Sticking during compression");
            // System.out.print(".");
            u_fin.setX(0.0);
            u_fin.setY(0.0);    // ux=uy=0.0 if stably stuck.

            // System.out.println("Wz, uz: " + Wz + ", " + uz);
            if (uz < 0.0)    // Only do this if still compressing.  If extending, then just throw the energy away...
               Wz = Wz + 0.5 * K_inv.getElement(2, 2) * (0 - uz * uz);    // Wz(b) = Wz(a) + 1/2 K_inv(3,3)*(b^2-a^2);

            // System.out.println("epsilon: " + epsilon);

            Wz = Wz * epsilon * epsilon;

            if (Wz < 0.0)
            {
               uz = Math.sqrt(0.0 + 2.0 * (0.0 - Wz) / K_inv.getElement(2, 2));
            }    // Math.sqrt(uz*uz + 2.0*(0.0 - Wz)/K_inv.getElement(2,2));}
            else
               uz = 0.0;

            // System.out.println("Wz, uz: " + Wz + ", " + uz);

            u_fin.setZ(uz);

            // output[3] = 0.0;

         }
         else
         {
//             System.out.println("Unstable Sticking during compression and restitution!");
            // Mirtich p.76

            if (uz > 0.0)
            {
               uz = 0.0;
            }    // If extending, then just throw the energy away... +++JEP???

            double beta = solveBeta(K, mu);
            zeta_B.setX(-mu * Math.cos(beta));
            zeta_B.setY(-mu * Math.sin(beta));
            zeta_B.setZ(1.0);

            // System.out.println("beta: " + beta);

            // Compute at bottom:

            double ux_bot = ux + Kx.dot(zeta_B) / Kz.dot(zeta_B) * (0.0 - uz);
            double uy_bot = uy + Ky.dot(zeta_B) / Kz.dot(zeta_B) * (0.0 - uz);
            double Wz_bot = Wz + (0.0 * 0.0 - uz * uz) / (2.0 * Kz.dot(zeta_B));

            // System.out.println("At bottom: (ux, uy, uz, Wz) = (" + ux_bot + ", " + uy_bot + ", 0.0, " + Wz_bot + ")" );

            u_fin.setZ(Math.sqrt(0.0 * 0.0 + 2.0 * Kz.dot(zeta_B) * (0.0 - Wz_bot)));
            u_fin.setX(ux_bot + Kx.dot(zeta_B) / Kz.dot(zeta_B) * (u_fin.getZ() - 0.0));
            u_fin.setY(uy_bot + Ky.dot(zeta_B) / Kz.dot(zeta_B) * (u_fin.getZ() - 0.0));

            // System.out.println("u_fin: " + u_fin);

         }
      }
      else
      {
         // System.out.println("Not Stuck after compression.  Doing restitution integration");
         // System.out.println("Before restitution ux: " + ux + ", uy: " + uy + ", uz: " + uz + ", Wz: " + Wz);

         // If Wz > 0.0, then something went wrong here (Integrated too far).  If so, then set Wz = 0.0; +++JEP
         if (Wz > 0.0)
            Wz = 0.0;

         integrateRestitution(ux, uy, uz, Wz * epsilon * epsilon, restitution_output);
         ux = restitution_output[0];
         uy = restitution_output[1];
         uz = restitution_output[2];
         Wz = restitution_output[3];

         // System.out.println("after restitution ux: " + ux + ", uy: " + uy + ", uz: " + uz + ", Wz: " + Wz);
         if (amStuck)
         {
            // If Wz > 0.0, then something went wrong here (Integrated too far).  If so, then set Wz = 0.0; +++JEP
            if (Wz > 0.0)
               Wz = 0.0;

            if (this.stableSticking)
            {
               // Stable Sticking during Restitution.  Mirtich p. 75;
               // System.out.println("Stable Sticking during restitution");
               // System.out.print(".");

               u_fin.setX(0.0);
               u_fin.setY(0.0);    // ux=uy=0.0 if stably stuck.

               if (Wz < 0.0)
               {
                  uz = Math.sqrt(uz * uz + 2.0 * (0.0 - Wz) / K_inv.getElement(2, 2));
               }
               else
               {
                  uz = 0.0;
               }

               u_fin.setZ(uz);

               // output[3] = 0.0;

            }
            else
            {
               // System.out.println("Un-stable Sticking during restitution!");
               // Mirtich p.76

               double beta = solveBeta(K, mu);
               zeta_B.setX(-mu * Math.cos(beta));
               zeta_B.setY(-mu * Math.sin(beta));
               zeta_B.setZ(1.0);

               // Restitution:
               u_fin.setZ(Math.sqrt(uz * uz + 2.0 * Kz.dot(zeta_B) * (0.0 - Wz)));
               u_fin.setX(ux + Kx.dot(zeta_B) / Kz.dot(zeta_B) * (u_fin.getZ() - 0.0));
               u_fin.setY(uy + Ky.dot(zeta_B) / Kz.dot(zeta_B) * (u_fin.getZ() - 0.0));

            }
         }
         else
         {
            // System.out.print(".");
            // System.out.println("Not Stuck after restitution");
            u_fin.setX(ux);
            u_fin.setY(uy);
            u_fin.setZ(uz);

            // System.out.println("u_final: " + u_fin);
         }
      }

      // Check to make sure that after collision, uz is positive:
      if (u_fin.getZ() < 0.0)
      {
         u_fin.setZ(0.0);

         // System.out.println("uz still negative after collision!!!!");
      }
   }

   private double[] coeffs = new double[5];
   private double[] solutions = new double[4];
   private QuarticRootFinder rootFinder = new QuarticRootFinder();

   private double solveBeta(Matrix3D K, double mu)
   {
      // Mirtich p. 83;
//      @SuppressWarnings("unused")
      double a = K.getM00(), b = K.getM11(), c = K.getM22(), d = K.getM12(), e = K.getM02(), f = K.getM01();

      coeffs[0] = -f * mu + d;
      coeffs[1] = 2.0 * mu * (a - b) - 2.0 * e;
      coeffs[2] = 6.0 * f * mu;
      coeffs[3] = -2.0 * mu * (a - b) - 2.0 * e;
      coeffs[4] = -f * mu - d;

      double num_real_solutions = rootFinder.SolveQuartic(coeffs, solutions);

      // if (num_real_solutions == 2)
      // {
      // Find the diverging ray:
      for (int i = 0; i < num_real_solutions; i++)
      {
         // System.out.println("Solution " + i + " : " + solutions[i]);
         double t = solutions[i];
         if ((-a * mu - e) * t * t * t * t + (4.0 * f * mu + 2.0 * d) * t * t * t + (2.0 * a * mu - 4.0 * b * mu) * t * t + (-4.0 * f * mu + 2.0 * d) * t
                 + (-a * mu + e) > 0.0)
         {
            // System.out.println("Final solutions for t is:  " + t);
            return Math.atan2(2.0 * t, 1.0 - t * t);    // Math.tan(t)*2.0;
         }
      }

      // }

      /*
       * System.err.println("Error!! No diverging solutions for beta!!");
       * System.err.println("Number of Real solutions is : " + num_real_solutions);
       * System.err.println("K is:  " + K);
       * System.err.println("mu is:  " + mu);
       * System.err.println("Solutions for t are:  " + solutions);
       */



      return 0.0;
   }


   private CollisionRungeKutta collisionRungeKutta;    // = new CollisionRungeKutta(3);
   private CollisionRungeKutta pzRungeKutta;

   double[] rk_input = new double[4];
   double[] rk_range = new double[2];

   PZDerivativeVector pzDerivativeVector = new PZDerivativeVector();

   private void integrateWRTpz(double ux, double uy, double uz, double Wz, double step_size, double[] output)
   {
      rk_input[0] = ux;
      rk_input[1] = uy;
      rk_input[2] = uz;
      rk_input[3] = Wz;
      rk_range[0] = 0.0;
      rk_range[1] = step_size;

      try
      {
         pzRungeKutta.integrate(rk_input, rk_range, pzDerivativeVector);    // From 0.0 to uz or from uz to 0.0???
      }
      catch (ODEException e)
      {
         System.out.println("Exception in integrateWRTpz: " + e);
      }
      catch (CollisionDerivativeException e)
      {
         System.out.println("Exception in integrateWRTpz: " + e);
      }

      output[0] = rk_input[0];
      output[1] = rk_input[1];    // uy;
      output[2] = rk_input[2];    // uz;
      output[3] = rk_input[3];    // Wz;
   }


   CompressionDerivativeVector compressionDerivativeVector = new CompressionDerivativeVector();

   private void integrateCompression(double ux, double uy, double uz, double Wz, double[] output) throws CollisionDerivativeException
   {
      // Output is [ux, uy, uz, Wz].

      // double ux = u0.x, uy = u0.y, uz = u0.z;
      // double Wz = 0.0;

      /*
       * double ux = input[0], uy = input[1], uz = input[2];
       * double Wz = input[3];
       */

      // System.out.println("Wz: " + Wz + ", ux: " + ux + ", uy: " + uy + ", uz: " + uz);

      // Integrate during the compression stage from uz to a little past zero.

      rk_input[0] = ux;
      rk_input[1] = uy;
      rk_input[2] = Wz;
      rk_range[0] = uz;
      rk_range[1] = UZ_OVERSHOOT;


      try
      {
         collisionRungeKutta.integrate(rk_input, rk_range, compressionDerivativeVector);    // From 0.0 to uz or from uz to 0.0???
      }
      catch (ODEException e)
      {
         System.out.println("Exception in integrateCompression: " + e);

         // System.out.println("K: " + K);
         // System.out.println("u0: " + u0);
         // System.out.println("epsilon: " + epsilon);
         // System.out.println("mu: " + mu);

         System.exit(0);
      }

      /*
       * catch(CollisionDerivativeException e)
       * {
       * System.out.println("Exception in integrateCompression: " + e);
       * // +++JEP.  This happens when we should be integrating with respect to pz.  What should we do???
       * }
       */


      output[0] = rk_input[0];    // ux
      output[1] = rk_input[1];    // uy;
      output[2] = rk_range[1];    // uz ends at 0.0;
      output[3] = rk_input[2];    // Wz;

   }

   RestitutionDerivativeVector restitutionDerivativeVector = new RestitutionDerivativeVector();

   private void integrateRestitution(double ux, double uy, double uz, double Wz, double[] output)
   {
      // double ux = input[0], uy = input[1], uz = input[2], Wz=input[3];
      // System.out.println("Wz: " + Wz + ", ux: " + ux + ", uy: " + uy + ", uz: " + uz);

      // Integrate during the restitution stage until Wz goes to zero.

      rk_input[0] = ux;
      rk_input[1] = uy;
      rk_input[2] = uz;
      rk_range[0] = Wz;
      rk_range[1] = 0.0;

      try
      {
         collisionRungeKutta.integrate(rk_input, rk_range, restitutionDerivativeVector);    // From 0.0 to uz or from uz to 0.0???
      }
      catch (ODEException e)
      {
         System.out.println("Exception in integrateRestitution: " + e);
         System.out.println("Before integration:  (ux,uy,uz,Wz) = (" + ux + ", " + uy + ", " + uz + ", " + Wz + ")");
         System.out.println("At exception point:  (ux,uy,uz,Wz) = (" + rk_input[0] + ", " + rk_input[1] + ", " + rk_input[2] + ", " + "???" + ")");

         System.out.println("K = " + K);
         System.out.println("u0 = " + u0);
         System.out.println("epsilon = " + epsilon);
         System.out.println("mu = " + mu);

      }
      catch (CollisionDerivativeException e)
      {
         System.out.println("Exception in integrateRestitution: " + e);
      }



      output[0] = rk_input[0];    // ux
      output[1] = rk_input[1];    // uy;
      output[2] = rk_input[2];    // uz;
      output[3] = rk_range[1];    // Wz;

   }

   Vector3D zeta = new Vector3D();

   @SuppressWarnings("serial")
   private class PZDerivativeVector implements CollisionDerivativeVector
   {
      public PZDerivativeVector()
      {
      }

      @Override
      public void derivs(double pz, double[] state, double[] deriv)
      {
         @SuppressWarnings("unused")
         double ux = state[0], uy = state[1], uz = state[2], Wz = state[3];
         double utot = Math.sqrt(ux * ux + uy * uy);

         {
            // System.out.println("ux, uy, uz, Wz: " + ux + ", " + uy + ", " + uz + ", " + Wz);
            zeta.set(-mu * ux / utot, -mu * uy / utot, 1.0);

            // Derivative during compression step.  Mirtich p. 72

            deriv[0] = Kx.dot(zeta);
            deriv[1] = Ky.dot(zeta);
            deriv[2] = Kz.dot(zeta);
            deriv[3] = uz;
         }
      }

      @Override
      public boolean isStuck(double[] state)
      {
         double ux = state[0], uy = state[1];
         double utot = Math.sqrt(ux * ux + uy * uy);

         if (utot < U_STUCK_THRESH)
         {
            // System.out.println("Stuck during pz integration");
            // System.out.println("ux, uy, uz, Wz: " + ux + ", " + uy + ", " + state[2] + ", " + state[3]);
            amStuck = true;

            return true;
         }

         return false;
      }
   }


   private class CompressionDerivativeVector implements CollisionDerivativeVector
   {
      private static final long serialVersionUID = -4133081308328360568L;

      public CompressionDerivativeVector()
      {
      }


      @Override
      public void derivs(double uz, double[] state, double[] deriv) throws CollisionDerivativeException
      {
         @SuppressWarnings("unused")
         double ux = state[0], uy = state[1], Wz = state[2];
         double utot = Math.sqrt(ux * ux + uy * uy);

         {
            // System.out.println("ux, uy, uz, Wz: " + ux + ", " + uy + ", " + uz + ", " + Wz);
            zeta.set(-mu * ux / utot, -mu * uy / utot, 1.0);

            // Derivative during compression step.  Mirtich p. 72

            // System.out.println("Kz.dot(zeta): " + Kz.dot(zeta));
            double Kz_zeta_inv = 1.0 / (Kz.dot(zeta));

            // NOTE: Kz_zeta_inv must be greater than 0.0 for Wz to decrease!!

            if (Kz_zeta_inv < 0.0)
            {
               // hadBadCompression = true;
               // System.out.println("Had Bad Compression.  Should stop...");
               throw new CollisionDerivativeException("Bad Compression.  Kz_zeta_inv = " + Kz_zeta_inv);

               // System.out.println("Kz_zeta_inv: " + Kz_zeta_inv);
               // System.out.println("zeta: " + zeta);
            }

            deriv[0] = Kx.dot(zeta) * Kz_zeta_inv;
            deriv[1] = Ky.dot(zeta) * Kz_zeta_inv;
            deriv[2] = uz * Kz_zeta_inv;
         }
      }

      @Override
      public boolean isStuck(double[] state)
      {
         double ux = state[0], uy = state[1];
         double utot = Math.sqrt(ux * ux + uy * uy);

         if (utot < U_STUCK_THRESH)
         {
            amStuck = true;
            return true;
         }
         // System.out.println("Checking if stuck during compression.  (ux, uy): ("+ux+","+uy+")");

         return false;
      }
   }


   private class RestitutionDerivativeVector implements CollisionDerivativeVector
   {
      private static final long serialVersionUID = 4537460728198035585L;

      public RestitutionDerivativeVector()
      {
      }

      @Override
      public void derivs(double Wz, double[] state, double[] deriv)
      {
         double ux = state[0], uy = state[1], uz = state[2];
         double utot = Math.sqrt(ux * ux + uy * uy);

         {
            // System.out.println("ux, uy, uz, Wz: " + ux + ", " + uy + ", " + uz + ", " + Wz);
            zeta.set(-mu * ux / utot, -mu * uy / utot, 1.0);

            // Derivative during compression step.  Mirtich p. 72

            deriv[0] = Kx.dot(zeta) / uz;    // ux_dot  = Kx dot  zeta/uz;
            deriv[1] = Ky.dot(zeta) / uz;    // uy_dot  = Ky dot  zeta/uz;
            deriv[2] = Kz.dot(zeta) / uz;    // uz_dot  = Ky dot  zeta/uz;

         }
      }

      @Override
      public boolean isStuck(double[] state)
      {
         double ux = state[0], uy = state[1];
         double utot = Math.sqrt(ux * ux + uy * uy);

         if (utot < U_STUCK_THRESH)
         {
            amStuck = true;
            return true;
         }

         // System.out.println("Checking if stuck during restitution");
         return false;
      }
   }
}
