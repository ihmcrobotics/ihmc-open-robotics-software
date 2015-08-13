package us.ihmc.exampleSimulations.collisionExample;


import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.CollisionIntegrator;


public class CollisionExampleTest1
{
   double mu = 0.92, epsilon = 0.0;
   double M1 = 1.0, GYR1 = 0.5, R1 = 0.2;
   double M2 = 1.0, GYR2 = 0.5, R2 = 0.2;

   Vector3d x1_world = new Vector3d(0.0, 0.0, 0.0), x2_world;
   Vector3d u1_world = new Vector3d(1.0, 2.0, 3.0), u2_world = new Vector3d(0.0, 0.0, 0.0);
   double xRotation = Math.PI / 2.0;

   Matrix3d R_world_collision = new Matrix3d(), R_collision_world;


   double Ixx1 = M1 * (R1 * GYR1) * (R1 * GYR1), Iyy1 = M1 * (R1 * GYR1) * (R1 * GYR1), Izz1 = M1 * (R1 * GYR1) * (R1 * GYR1);
   double Ixx2 = M2 * (R2 * GYR2) * (R2 * GYR2), Iyy2 = M2 * (R2 * GYR2) * (R2 * GYR2), Izz2 = M2 * (R2 * GYR2) * (R2 * GYR2);

   Matrix3d I1_world = new Matrix3d(Ixx1, 0, 0, 0, Iyy1, 0, 0, 0, Izz1);
   Matrix3d I2_world = new Matrix3d(Ixx2, 0, 0, 0, Iyy2, 0, 0, 0, Izz2);

   Matrix3d I1, I2;
   Vector3d r1, r2;
   Vector3d u1, u2, u;

   Vector3d e_world, r1_world, r2_world;

   Matrix3d r1_twidle, r2_twidle, K, K_inverse;

   Vector3d u_final, impulse;

   Vector3d impulse_world;
   Vector3d delta_u, delta_u_world, delta_u1_world, delta_w1_world, delta_u2_world, delta_w2_world;

   Vector3d momentum_world_before, momentum_world_after;

   public CollisionExampleTest1()
   {
      momentum_world_before = new Vector3d(u1_world);
      momentum_world_before.scale(M1);
      Vector3d temp = new Vector3d(u2_world);
      temp.scale(M2);
      momentum_world_before.add(temp);

      double energy_world_before = 0.5 * u1_world.dot(u1_world) + 0.5 * u2_world.dot(u2_world);

      R_world_collision.rotX(xRotation);

      Vector3d offset_world = new Vector3d(0.0, 0.0, R1 + R2);
      R_world_collision.transform(offset_world);

      x2_world = new Vector3d(x1_world);
      x2_world.sub(offset_world);

      // Find intersection point and offset vectors:

      e_world = new Vector3d(x2_world);
      e_world.sub(x1_world);

      r1_world = new Vector3d(e_world);
      r1_world.scale(R1 / (R1 + R2));
      r2_world = new Vector3d(e_world);
      r2_world.scale(-R2 / (R1 + R2));

      // Rotate everything into collision frame:

      R_collision_world = new Matrix3d(R_world_collision);
      R_collision_world.transpose();

      I1 = new Matrix3d();
      I1.mul(R_collision_world, I1_world);
      I1.mul(R_world_collision);

      I2 = new Matrix3d();
      I2.mul(R_collision_world, I2_world);
      I2.mul(R_world_collision);

      u1 = new Vector3d(u1_world);
      R_collision_world.transform(u1);

      u2 = new Vector3d(u2_world);
      R_collision_world.transform(u2);


      u = new Vector3d(u1);
      u.sub(u2);


      r1 = new Vector3d(r1_world);
      R_collision_world.transform(r1);
      r2 = new Vector3d(r2_world);
      R_collision_world.transform(r2);


      // Compute the K Matrix:


      r1_twidle = new Matrix3d(0.0, -r1.z, r1.y, r1.z, 0.0, -r1.x, -r1.y, r1.x, 0.0);
      r2_twidle = new Matrix3d(0.0, -r2.z, r2.y, r2.z, 0.0, -r2.x, -r2.y, r2.x, 0.0);


      Matrix3d I1_inv = new Matrix3d(I1);
      I1_inv.invert();
      Matrix3d I2_inv = new Matrix3d(I2);
      I2_inv.invert();

      Matrix3d t1 = new Matrix3d(r1_twidle);
      t1.mul(I1_inv);
      t1.mul(r1_twidle);
      Matrix3d t2 = new Matrix3d(r2_twidle);
      t2.mul(I2_inv);
      t2.mul(r2_twidle);

      K = new Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1);
      K.mul(1.0 / M1 + 1.0 / M2);
      K.sub(t1);
      K.sub(t2);

      K_inverse = new Matrix3d(K);
      K_inverse.invert();

      // Report:

      System.out.println("x1_world, x2_world:");
      System.out.println(x1_world);
      System.out.println(x2_world);
      System.out.println("r1_world, r2_world:");
      System.out.println(r1_world);
      System.out.println(r2_world);
      System.out.println("u1_world, u2_world:");
      System.out.println(u1_world);
      System.out.println(u2_world);


      System.out.println("r1, r2:");
      System.out.println(r1);
      System.out.println(r2);
      System.out.println("u1, u2:");
      System.out.println(u1);
      System.out.println(u2);

      System.out.println("u:");
      System.out.println(u);
      System.out.println("K:");
      System.out.println(K);


      // Integrate the collision:

      CollisionIntegrator collisionIntegrator = new CollisionIntegrator();
      collisionIntegrator.setup(K, u, epsilon, mu);

//    double[] output = new double[4];
      Vector3d output = new Vector3d();
      collisionIntegrator.integrate(output);

      u_final = new Vector3d(output);


      System.out.println("u_final:");
      System.out.println(u_final);

      // Compute the impulse:

      delta_u = new Vector3d(u);
      delta_u.sub(u_final);
      System.out.println("delta_u:");
      System.out.println(delta_u);

      delta_u_world = new Vector3d(delta_u);
      R_world_collision.transform(delta_u_world);
      System.out.println("delta_u_world:");
      System.out.println(delta_u_world);

      impulse = new Vector3d(delta_u);
      K_inverse.transform(impulse);
      System.out.println("impulse:");
      System.out.println(impulse);

      // Transform the impulse into world coordinates:

      impulse_world = new Vector3d(impulse);
      R_world_collision.transform(impulse_world);
      System.out.println("impulse_world:");
      System.out.println(impulse_world);

      // Figure out the change in velocities of the masses:

      delta_u1_world = new Vector3d(impulse_world);
      delta_u1_world.scale(1.0 / M1);
      u1_world.sub(delta_u1_world);

      delta_w1_world = new Vector3d();
      delta_w1_world.cross(r1_world, impulse_world);
      I1_inv.transform(delta_w1_world);


      delta_u2_world = new Vector3d(impulse_world);
      delta_u2_world.scale(-1.0 / M2);
      u2_world.sub(delta_u2_world);

      delta_w2_world = new Vector3d();
      delta_w2_world.cross(r2_world, impulse_world);
      I2_inv.transform(delta_w2_world);


      System.out.println("u1_world, u2_world after collision:  ");
      System.out.println(u1_world);
      System.out.println(u2_world);
      System.out.println("delta_w1_world, delta_w2_world after collision:  ");
      System.out.println(delta_w1_world);
      System.out.println(delta_w2_world);


      momentum_world_after = new Vector3d(u1_world);
      momentum_world_after.scale(M1);
      temp = new Vector3d(u2_world);
      temp.scale(M2);
      momentum_world_after.add(temp);

      System.out.println("Total momentum_world before, after:  ");
      System.out.println(momentum_world_before);
      System.out.println(momentum_world_after);

      double energy_world_after = 0.5 * u1_world.dot(u1_world) + 0.5 * u2_world.dot(u2_world);



      System.out.println("Total energy_world before, after:  ");
      System.out.println(energy_world_before);
      System.out.println(energy_world_after);


   }

   public static void main(String[] args)
   {
      CollisionExampleTest1 example = new CollisionExampleTest1();




   }
}
