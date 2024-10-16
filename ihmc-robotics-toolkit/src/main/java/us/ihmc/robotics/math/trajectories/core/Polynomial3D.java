package us.ihmc.robotics.math.trajectories.core;

import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.abstracts.AbstractPolynomial3D;

import java.util.Arrays;
import java.util.List;

public class Polynomial3D extends AbstractPolynomial3D
{
   public Polynomial3D(int maximumNumberOfCoefficients)
   {
      this(new Polynomial(maximumNumberOfCoefficients), new Polynomial(maximumNumberOfCoefficients), new Polynomial(maximumNumberOfCoefficients));
   }

   public Polynomial3D(Polynomial[] trajectory)
   {
      this(trajectory[0], trajectory[1], trajectory[2]);

      if (trajectory.length != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + trajectory.length
               + " YoTrajectories.");
   }

   public Polynomial3D(List<Polynomial> trajectory)
   {
      this(trajectory.get(0), trajectory.get(1), trajectory.get(2));

      if (trajectory.size() != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + trajectory.size()
               + " YoTrajectories.");
   }

   public Polynomial3D(Polynomial3D other)
   {
      super(other);
   }

   public Polynomial3D(Polynomial xTrajectory, Polynomial yTrajectory, Polynomial zTrajectory)
   {
      super(xTrajectory, yTrajectory, zTrajectory);
   }

   public static Polynomial3D[] createTrajectory3DArray(Polynomial[] xTrajectory, Polynomial[] yTrajectory, Polynomial[] zTrajectory)
   {
      return createTrajectory3DArray(Arrays.asList(xTrajectory), Arrays.asList(yTrajectory), Arrays.asList(zTrajectory));
   }

   public static List<Polynomial3D> createTrajectory3DList(Polynomial[] xTrajectory, Polynomial[] yTrajectory, Polynomial[] zTrajectory)
   {
      return Arrays.asList(createTrajectory3DArray(xTrajectory, yTrajectory, zTrajectory));
   }

   public static List<Polynomial3D> createTrajectory3DList(List<Polynomial> xTrajectory, List<Polynomial> yTrajectory, List<Polynomial> zTrajectory)
   {
      return Arrays.asList(createTrajectory3DArray(xTrajectory, yTrajectory, zTrajectory));
   }

   public static Polynomial3D[] createTrajectory3DArray(List<Polynomial> xTrajectory, List<Polynomial> yTrajectory, List<Polynomial> zTrajectory)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      Polynomial3D[] yoTrajectory3Ds = new Polynomial3D[xTrajectory.size()];

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds[i] = new Polynomial3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i));
      }
      return yoTrajectory3Ds;
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
