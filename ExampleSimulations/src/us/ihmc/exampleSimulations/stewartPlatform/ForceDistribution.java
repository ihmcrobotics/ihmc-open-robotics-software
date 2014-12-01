package us.ihmc.exampleSimulations.stewartPlatform;

import javax.vecmath.Vector3d;

import Jama.Matrix;

public class ForceDistribution
{
   public static void main(String args[])
   {
      double[] act_forces = new double[6];

      System.out.println(System.currentTimeMillis());

      for (int j = 0; j < 100000; j++)
      {
         /*
          * solveActuatorForces(act_forces, 100.0, 11.0,62.0,93.06,54.0,25.0,46.0,77.0,98.0,109.0,10.0,11.06,120.0,134.0,14.0,15.0,16.0,17.0,187.0,19.0,20.0,21.0,22.0,234.0,24.0,25.0,
          * 26.06,27.06,280.0,29.0,30.60,391.0,32.60,33.0,364.06,35.0,361.0);
          */
      }

      System.out.println(System.currentTimeMillis());
      System.out.println();

      /*
       * solveActuatorForces(act_forces, 100.0, 1.0,0.0,0.0,0.0,0.0,0.0,
       *                                      0.0,1.0,0.0,0.0,0.0,0.0,
       *                                      0.0,0.0,1.0,0.0,0.0,0.0,
       *                                      0.0,0.0,0.0,1.0,0.0,0.0,
       *                                      0.0,0.0,0.0,0.0,1.0,0.0,
       *                                      0.0,0.0,0.0,0.0,0.0,1.0);
       */

      for (int i = 0; i < 6; i++)
      {
         System.out.println(act_forces[i]);
      }

      System.out.println();

      double[] leg_forces = new double[3];

      // solveLegForces(leg_forces, 100.0, 0.0, 2.0, 3.0, 0.0, 3.0, 1.0);

      for (int i = 0; i < 3; i++)
      {
         System.out.println(leg_forces[i]);
      }

   }

   private Matrix aMatrix66 = new Matrix(6, 6);


   public void solveActuatorForcesSingleLeg(double[] act_force, double Fx, double Fy, double Fz, double Nx, double Ny, double Nz, Vector3d[] a_hat,
           Vector3d[] b)
   {
      for (int i = 0; i < 6; i++)
      {
         aMatrix66.set(0, i, a_hat[i].x);
         aMatrix66.set(1, i, a_hat[i].y);
         aMatrix66.set(2, i, a_hat[i].z);
         aMatrix66.set(3, i, b[i].x);
         aMatrix66.set(4, i, b[i].y);
         aMatrix66.set(5, i, b[i].z);

         // System.out.println(i + ": a_hat: " + a_hat[i]);
         // System.out.println("b: " + b[i]);

      }

      // System.out.println();



      double[][] vals2 =
      {
         {Fx}, {Fy}, {Fz}, {Nx}, {Ny}, {Nz}
      };
      Matrix bMatrix = new Matrix(vals2);

      double det = aMatrix66.det();

      // System.out.println(det);
      if (Math.abs(det) < 0.000001)
         return;

      Matrix x = aMatrix66.solve(bMatrix);


      for (int i = 0; i < 6; i++)
      {
         act_force[i] = x.get(i, 0);
      }
   }


}
