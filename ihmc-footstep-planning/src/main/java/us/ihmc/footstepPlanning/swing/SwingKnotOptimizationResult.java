package us.ihmc.footstepPlanning.swing;

import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.Arrays;

public class SwingKnotOptimizationResult
{
   private static int NUMBER_OF_KNOT_POINTS = 12;

   private Vector3D[] knotDisplacement = new Vector3D[NUMBER_OF_KNOT_POINTS];
   private short[] knotCollisions = new short[NUMBER_OF_KNOT_POINTS];
   private byte[] knotCollisionSeverity = new byte[NUMBER_OF_KNOT_POINTS];
   private float[] shiftDistances = new float[NUMBER_OF_KNOT_POINTS];

   private boolean verbose = true;

   private int numberOfCollisions = 0;
   private int numberOfSevereCollisoins = 0;

   public SwingKnotOptimizationResult()
   {
      reset();
   }

   public void reset()
   {
      for (int i = 0; i < NUMBER_OF_KNOT_POINTS; i++)
      {
         knotDisplacement[i] = new Vector3D();
         knotCollisions[i] = 0;
         knotCollisionSeverity[i] = 0;
      }
   }

   public void setKnotDisplacement(int index, Vector3D displacement)
   {
      knotDisplacement[index].add(displacement);
      shiftDistances[index] = (float) knotDisplacement[index].norm();
   }

   public void setKnotCollisions(int index, boolean collision)
   {
      if (collision)
      {
         knotCollisions[index]++;
         numberOfCollisions++;
      }
   }

   public void setKnotCollisionSeverity(int index, byte severity)
   {
      knotCollisionSeverity[index] = severity;
      if (severity > 0)
      {
         numberOfSevereCollisoins++;
      }
   }

   public String toString()
   {
      if (verbose)
      {
         String ret = "Collisions: " + Arrays.toString(knotCollisions) + "\t, Distances: " + Arrays.toString(shiftDistances);
         return ret;
      }
      else
      {
         return "Number of collisions: " + numberOfCollisions + ", Number of severe collisions: " + numberOfSevereCollisoins;
      }
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }
}
